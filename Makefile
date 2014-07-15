#.PATH: ${.CURDIR}/../../../../dev/sound/pci
.PATH: /usr/src/sys/dev/sound/pci

KMOD=	snd_xonar
SRCS=	device_if.h bus_if.h pci_if.h channel_if.h mixer_if.h
SRCS+=	xonar_compat.c xonar.c

.include <bsd.kmod.mk>
