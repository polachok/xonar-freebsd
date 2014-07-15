#include <sys/types.h>

#include <dev/sound/pcm/sound.h>
#include <sys/sysctl.h>
#include <sys/bus.h>

/* malloc / printf stuff */
extern int kern_printf(const char *fmt, ...);
extern int kern_snprintf(char *str, size_t size, const char *fmt, ...);
extern void* kern_malloc(unsigned long size, struct malloc_type *type, int flags);
extern void kern_free (void *addr, struct malloc_type *type);

/* dma tag creation also differs */
extern int xonar_create_dma_tag(bus_dma_tag_t *tag, bus_size_t maxsize, bus_dma_tag_t parent_tag, void *lock);

/* stuff related to pcm driver */
extern int pcm_sndbuf_alloc(struct snd_dbuf *b, bus_dma_tag_t dmatag, int dmaflags,
							unsigned int size);
extern struct sysctl_ctx_list* kern_sysctl_ctx(device_t dev); /* pcm driver manages sysctls in DragonFly */
extern struct sysctl_oid* kern_sysctl_tree(device_t dev);
