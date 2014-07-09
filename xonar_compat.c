#include <sys/types.h>
#include <sys/malloc.h>
#include <machine/stdarg.h>

#include "xonar_compat.h"

#if !defined(__DragonFly__) && !defined(__FreeBSD__)
#error "Platform not supported"
#endif

int
pcm_sndbuf_alloc(struct snd_dbuf *b, bus_dma_tag_t dmatag, int dmaflags,
					 unsigned int size)
{
#if defined __FreeBSD__
	return sndbuf_alloc(b, dmatag, dmaflags, size);
#elif defined __DragonFly__
	return sndbuf_alloc(b, dmatag, size);
#endif
}

int
kern_printf(const char *fmt, ...)
{
	int res;
#if defined __FreeBSD__
	va_list ap;
	va_start(ap, fmt);
	res = vprintf(fmt, ap);
	va_end(ap);
#elif defined __DragonFly__
	__va_list ap;
	__va_start(ap, fmt);
	res = kvprintf(fmt, ap);
	__va_end(ap);
#endif
	return res;
}

int
kern_snprintf(char *str, size_t size, const char *fmt, ...)
{
	int res;
#if defined __FreeBSD__
	va_list ap;
	va_start(ap, fmt);
	res = vsnprintf(str, size, fmt, ap);
	va_end(ap);
#elif defined __DragonFly__
	__va_list ap;
	__va_start(ap, fmt);
	res = kvsnprintf(str, size, fmt, ap);
	__va_end(ap);
#endif
	return res;
}

void*
kern_malloc(unsigned long size, struct malloc_type *type, int flags)
{
#if defined __FreeBSD__
	return malloc(size, type, flags);
#elif defined __DragonFly__
	return kmalloc(size, type, flags);
#endif
}

void
kern_free (void *addr, struct malloc_type *type)
{
#if defined __FreeBSD__
	return free(addr, type);
#elif defined __DragonFly__
	return kfree(addr, type);
#endif
}

int
xonar_create_dma_tag(bus_dma_tag_t *tag, bus_size_t maxsize, bus_dma_tag_t parent_tag, void *lock)
{
#if defined __FreeBSD__
	return bus_dma_tag_create( /* parent */ parent_tag,
							   /* alignment */ 4, /* boundary */ 0,
							   /* lowaddr */ BUS_SPACE_MAXADDR_32BIT,
							   /* highaddr */ BUS_SPACE_MAXADDR,
							   /* filter */ NULL, /* filterarg */ NULL,
							   /* maxsize */ maxsize, /* nsegments */ 1,
							   /* maxsegz */ 0x3ffff,
							   /* flags */ 0, /* lock fn */ busdma_lock_mutex,
							   /* lock */ lock, /* result */ tag);
#elif defined __DragonFly__
	return bus_dma_tag_create( /* parent */ parent_tag,
							   /* alignment */ 4, /* boundary */ 0,
							   /* lowaddr */ BUS_SPACE_MAXADDR_32BIT,
							   /* highaddr */ BUS_SPACE_MAXADDR,
							   /* filter */ NULL, /* filterarg */ NULL,
							   /* maxsize */ maxsize, /* nsegments */ 1,
							   /* maxsegz */ 0x3ffff,
							   /* flags */ 0, /* result */ tag);
#endif
}


struct sysctl_ctx_list*
kern_sysctl_ctx(device_t dev)
{
#if defined __FreeBSD__
	return device_get_sysctl_ctx(dev);
#elif defined __DragonFly__
	return snd_sysctl_tree(dev);
#endif
}

struct sysctl_oid* kern_sysctl_tree(device_t dev)
{
#if defined __FreeBSD__
	return device_get_sysctl_tree(dev);
#elif defined __DragonFly__
	return snd_sysctl_tree_top(dev);
#endif
}
