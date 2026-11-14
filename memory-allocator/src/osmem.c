// SPDX-License-Identifier: BSD-3-Clause

#include <string.h>
#include <unistd.h>
#include <sys/mman.h>

#include "osmem.h"
#include "block_meta.h"

#define MMAP_THRESHOLD	(128 * 1024)
#define ALIGNMENT	8
#define ALIGN(x)	(((x) + (ALIGNMENT - 1)) & ~((size_t)ALIGNMENT - 1))
#define META_SIZE	ALIGN(sizeof(struct block_meta))

/* Payload of a block <-> its metadata */
#define BLOCK_PAYLOAD(b)	((void *)((char *)(b) + META_SIZE))
#define PAYLOAD_BLOCK(p)	((struct block_meta *)((char *)(p) - META_SIZE))

/* Head of the list of heap (brk) blocks, ordered by address */
static struct block_meta *heap_head;

/* ------------------------------------------------------------------ */
/* Helpers                                                            */
/* ------------------------------------------------------------------ */

static struct block_meta *heap_tail(void)
{
	struct block_meta *b = heap_head;

	while (b && b->next)
		b = b->next;
	return b;
}

/* Extend the program break by `incr` bytes, die on failure */
static void *heap_grow(size_t incr)
{
	void *p = sbrk((intptr_t)incr);

	DIE(p == (void *)-1, "sbrk");
	return p;
}

/*
 * Split `b` so that it keeps exactly `size` bytes of payload.
 * The remainder becomes a new free block, but only if it can hold
 * a block_meta and at least 1 byte (aligned) of payload.
 */
static void split_block(struct block_meta *b, size_t size)
{
	struct block_meta *rest;

	if (b->size < size + META_SIZE + ALIGNMENT)
		return;

	rest = (struct block_meta *)((char *)BLOCK_PAYLOAD(b) + size);
	rest->size = b->size - size - META_SIZE;
	rest->status = STATUS_FREE;
	rest->prev = b;
	rest->next = b->next;
	if (b->next)
		b->next->prev = rest;
	b->next = rest;
	b->size = size;
}

/* Merge `b` with its next block (which must be free) */
static void merge_with_next(struct block_meta *b)
{
	struct block_meta *n = b->next;

	b->size += META_SIZE + n->size;
	b->next = n->next;
	if (n->next)
		n->next->prev = b;
}

/* Coalesce every run of adjacent free blocks on the heap */
static void coalesce_all(void)
{
	struct block_meta *b = heap_head;

	while (b && b->next) {
		if (b->status == STATUS_FREE && b->next->status == STATUS_FREE)
			merge_with_next(b);
		else
			b = b->next;
	}
}

/* Best-fit: smallest free block that is big enough */
static struct block_meta *find_best(size_t size)
{
	struct block_meta *b, *best = NULL;

	for (b = heap_head; b; b = b->next) {
		if (b->status != STATUS_FREE || b->size < size)
			continue;
		if (!best || b->size < best->size)
			best = b;
	}
	return best;
}

/* Allocate a block with brk (heap), preallocating on the first use */
static struct block_meta *heap_alloc(size_t size)
{
	struct block_meta *b, *last;

	/* Heap preallocation: happens only once */
	if (!heap_head) {
		b = heap_grow(MMAP_THRESHOLD);
		b->size = MMAP_THRESHOLD - META_SIZE;
		b->status = STATUS_FREE;
		b->prev = NULL;
		b->next = NULL;
		heap_head = b;
	} else {
		coalesce_all();
	}

	b = find_best(size);
	if (!b) {
		last = heap_tail();
		if (last->status == STATUS_FREE) {
			/* Expand the last free block */
			heap_grow(size - last->size);
			last->size = size;
			b = last;
		} else {
			/* Append a brand new block */
			b = heap_grow(META_SIZE + size);
			b->size = size;
			b->prev = last;
			b->next = NULL;
			last->next = b;
		}
	}

	split_block(b, size);
	b->status = STATUS_ALLOC;
	return b;
}

/* Allocate a block with mmap */
static struct block_meta *map_alloc(size_t size)
{
	struct block_meta *b;

	b = mmap(NULL, META_SIZE + size, PROT_READ | PROT_WRITE,
		 MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	DIE(b == MAP_FAILED, "mmap");

	b->size = size;
	b->status = STATUS_MAPPED;
	b->prev = NULL;
	b->next = NULL;
	return b;
}

/* Common allocation path; `threshold` decides brk vs mmap */
static void *do_alloc(size_t size, size_t threshold)
{
	struct block_meta *b;

	if (size == 0)
		return NULL;

	size = ALIGN(size);
	if (META_SIZE + size >= threshold)
		b = map_alloc(size);
	else
		b = heap_alloc(size);

	return BLOCK_PAYLOAD(b);
}

/* ------------------------------------------------------------------ */
/* Public API                                                         */
/* ------------------------------------------------------------------ */

void *os_malloc(size_t size)
{
	return do_alloc(size, MMAP_THRESHOLD);
}

void os_free(void *ptr)
{
	struct block_meta *b;
	int ret;

	if (!ptr)
		return;

	b = PAYLOAD_BLOCK(ptr);
	if (b->status == STATUS_MAPPED) {
		ret = munmap(b, META_SIZE + b->size);
		DIE(ret == -1, "munmap");
	} else if (b->status == STATUS_ALLOC) {
		b->status = STATUS_FREE;
	}
}

void *os_calloc(size_t nmemb, size_t size)
{
	size_t total;
	void *ptr;

	if (nmemb == 0 || size == 0)
		return NULL;

	/* Overflow check */
	if (nmemb > (size_t)-1 / size)
		return NULL;
	total = nmemb * size;

	ptr = do_alloc(total, (size_t)getpagesize());
	if (ptr)
		memset(ptr, 0, total);
	return ptr;
}

void *os_realloc(void *ptr, size_t size)
{
	struct block_meta *b;
	void *new_ptr;
	size_t old_size;

	if (!ptr)
		return os_malloc(size);

	if (size == 0) {
		os_free(ptr);
		return NULL;
	}

	b = PAYLOAD_BLOCK(ptr);
	if (b->status == STATUS_FREE)
		return NULL;

	size = ALIGN(size);

	/* Heap block: try to resize in place first */
	if (b->status == STATUS_ALLOC && META_SIZE + size < MMAP_THRESHOLD) {
		/* Shrink (or same size) */
		if (size <= b->size) {
			split_block(b, size);
			return ptr;
		}

		/* Expand by absorbing following free blocks, one at a time */
		while (b->next && b->next->status == STATUS_FREE) {
			merge_with_next(b);
			if (size <= b->size) {
				split_block(b, size);
				return ptr;
			}
		}

		/* Last block on the heap: grow it with brk */
		if (!b->next) {
			heap_grow(size - b->size);
			b->size = size;
			return ptr;
		}
	}

	/* Fallback: allocate elsewhere, copy, release the old block */
	new_ptr = os_malloc(size);
	if (!new_ptr)
		return NULL;

	old_size = b->size;
	memcpy(new_ptr, ptr, old_size < size ? old_size : size);
	os_free(ptr);
	return new_ptr;
}
