#
#        -Wall -W -Wbad-function-cast -Wcast-qual \
#	-Wstrict-prototypes \
#	-Wmissing-prototypes -Wmissing-declarations -Wredundant-decls \
#
#       $Id: Makefile,v 1.30 2000/09/28 19:57:01 ags-sw Exp ags-sw $
#
BUILDROOTDIR = ../buildroot
STAGING_DIR = $(BUILDROOTDIR)/output/host/usr/x86_64-buildroot-linux-gnu/sysroot
TOOLDIR = $(BUILDROOTDIR)/output/host/usr/bin/x86_64-buildroot-linux-gnu
BUILDROOTFSDIR = $(BUILDROOTDIR)/board/agslaser/rootfs_overlay
BUILDROOTTGTDIR = $(BUILDROOTDIR)/output/target
AGSCFGDIR = ../ags-config-files
LNXHDRDIR = ../linux_headers
CC=$(TOOLDIR)-gcc
LD=$(TOOLDIR)-ld
AS=$(TOOLDIR)-as
AR=$(TOOLDIR)/x86_64-buildroot-linux-uclibc-ar

# COMPILE AND LINK OPTIONS AND DEPENDENCIES
LDFLAGS=-Wl,--sysroot=$(STAGING_DIR) -Wl,--error-poison-system-directories -L$(STAGING_DIR)/lib -L$(STAGING_DIR)/usr/lib -lc -lm -lpthread
EXTRA_CFLAGS = -DAGS_DEBUG
USERINCLUDE    := \
	-I./ \
	-I$(LNXHDRDIR)/include
CFLAGS = -g -DLASER_DEFINED -march=atom  -Wall -Wmissing-prototypes -Wstrict-prototypes -Wunused -Werror $(USERINCLUDE)
AGS_OBJECTS = Main.o \
	      L3DTransform.o \
	      AngleCorrections.o \
              APTParser.o \
              CRCHandler.o  \
              comm_loop.o  \
              parse_data.o  \
	      DoAutoFocusCmd.o \
              LaserCmds.o \
	      LaserPattern.o \
	      LaserInterface.o \
              BoardComm.o Events.o \
              SensorRegistration.o \
              3DTransform.o \
              QuickCheckManager.o \
	      SystemSpecifics.o Video.o Web.o \
	      Init.o Net.o FOM.o \
	      angles.o chisqr.o amoeba.o \
              ShowTargets.o \
	      Hobbs.o \
	      SetBit.o \
	      RightOnFullReg.o RightOnCert.o DoFindOneTarget.o \
	      FullRegWithFeedback.o \
	      ParseAutoFocus.o \
	      ParseVisionFocus.o \
	      LaserFlex.o \
	      FlexCalculateTransform.o \
	      FlexCalWithFeedback.o  \
	      FlexRegWithFeedback.o \
	      DoTakePicture.o \
	      RefreshRate.o \
	      ChangeDisplayPeriod.o \
	      ChangeTransformTolerance.o \
	      GetTargetsUsed.o \
	      DoCoarseScan.o \
	      DoCoarseScan2.o \
	      CalibXY.o \
	      segtest.o segpoly.o pnpoly.o tan_init.o \
	      area_order.o heap.o allace4.o shoelace4.o \
	      CalculateTransform.o \
	      ParseVision.o \
	      RemoteSerial.o \
	      L2VtakePicture.o \
	      SensorSearch.o  Files.o
#
#
agsd : $(AGS_OBJECTS) BoardComm.h $(LNXHDRDIR)/include/linux/laser_api.h
	$(CC) $(LDFLAGS) -o $@ $(AGS_OBJECTS)

jigqc : $(JIG_OBJECTS) jigqc.o
	$(CC) jigqc.o -o jigqc ${JIG_OBJECTS} -g -lm /usr/lib/libncurses.a -static

oneside : $(AGS_OBJECTS) oneside.o
	$(CC) oneside.o -o oneside ${AGS_OBJECTS} fork.CTB.o SensorSearch.o -g -lm /usr/lib/libncurses.a -static

%.o: %.c BoardComm.h parse_data.h $(LNXHDRDIR)/include/linux/laser_api.h
	$(CC) -c -o $@ $(CFLAGS) $(EXTRA_CFLAGS) $<

jigqc.o: jigqc.c
	$(CC) -c -o jigqc.o $(CFLAGS) $(EXTRA_CFLAGS) jigqc.c

oneside.o: oneside.c
	$(CC) -c -o oneside.o $(CFLAGS) $(EXTRA_CFLAGS) oneside.c
clean:
	rm -rf *.o agsd $(BUILDROOTFSDIR)/*
usb_install:
	chmod 777 $(AGSCFGDIR)/S95agsd
	cp $(AGSCFGDIR)/S95agsd $(BUILDROOTTGTDIR)/etc/init.d
	chmod 777 $(AGSCFGDIR)/S50sshd
	cp $(AGSCFGDIR)/S50sshd $(BUILDROOTTGTDIR)/etc/init.d
	cp $(AGSCFGDIR)/sshd_config $(BUILDROOTTGTDIR)/etc/ssh
	cp $(AGSCFGDIR)/usb_fstab $(BUILDROOTTGTDIR)/etc/fstab
	cp $(AGSCFGDIR)/gdbinit $(BUILDROOTFSDIR)/.gdbinit
	mkdir $(BUILDROOTFSDIR)/laservision
	cp $(AGSCFGDIR)/skeleton.mk $(BUILDROOTDIR)/package/skeleton/
	cp $(AGSCFGDIR)/ags-busybox-config $(BUILDROOTDIR)/package/busybox
	cp $(AGSCFGDIR)/ags-buildroot-config $(BUILDROOTDIR)/.config
mmc_install:
	chmod 777 $(AGSCFGDIR)/S95agsd
	cp $(AGSCFGDIR)/S95agsd $(BUILDROOTTGTDIR)/etc/init.d
	chmod 777 $(AGSCFGDIR)/S50sshd
	cp $(AGSCFGDIR)/S50sshd $(BUILDROOTTGTDIR)/etc/init.d
	cp $(AGSCFGDIR)/sshd_config $(BUILDROOTTGTDIR)/etc/ssh
	cp $(AGSCFGDIR)/mmc_fstab $(BUILDROOTTGTDIR)/etc/fstab
	cp $(AGSCFGDIR)/gdbinit $(BUILDROOTFSDIR)/.gdbinit
	mkdir $(BUILDROOTFSDIR)/laservision
	cp $(AGSCFGDIR)/skeleton.mk $(BUILDROOTDIR)/package/skeleton/
	cp $(AGSCFGDIR)/ags-busybox-config $(BUILDROOTDIR)/package/busybox
	cp $(AGSCFGDIR)/ags-buildroot-config $(BUILDROOTDIR)/.config

# burnusb copies rootfs to boot partition and ALL files to data partition
burnusb:
	sudo umount /dev/sdb1
	sudo umount /dev/sdb2
	sudo mkdir /mnt/lvboot
	sudo mkdir /mnt/lvdata
	sudo mount /dev/sdb1 /mnt/lvboot
	sudo mount /dev/sdb2 /mnt/lvdata
	chmod 777 agsd
	cp agsd /mnt/lvdata/sbin/agsd
	sudo mount -o loop,ro $(BUILDROOTDIR)/output/images/rootfs.ext2 $(BUILDROOTDIR)/output/ext2
	sudo cp -avrf $(BUILDROOTDIR)/output/ext2/* /mnt/lvboot
	sudo cp $(BUILDROOTDIR)/output/images/bzImage /mnt/lvboot
	sudo cp $(AGSCFGDIR)/extlinux.conf /mnt/lvboot
	sudo umount /dev/sdb1
	sudo umount /dev/sdb2
	sudo rm -rf /mnt/lvdata
	sudo rm -rf /mnt/lvboot
	sudo umount $(BUILDROOTDIR)/output/ext2

# newusb copies rootfs to boot partition and ALL files to data partition
newusb:
	sudo umount /dev/sdb1
	sudo mkdir /mnt/lvboot
	sudo mkdir /mnt/lvdata
	sudo mount /dev/sdb1 /mnt/lvboot
	sudo mount /dev/sdb2 /mnt/lvdata
	sudo mkdir /mnt/lvdata/laservision
	sudo mkdir /mnt/lvdata/laservision/data
	sudo mkdir /mnt/lvdata/laservision/sbin
	sudo chmod 777 agsd
	sudo cp agsd /mnt/lvdata/laservision/sbin
	sudo cp -avrf $(AGSCFGDIR)/autofocus.txt /mnt/lvdata/laservision/data/autofocus
	sudo dos2unix /mnt/lvdata/laservision/data/autofocus
	sudo cp -avrf $(AGSCFGDIR)/visionfocus.txt /mnt/lvdata/laservision/data/focusvision
	sudo dos2unix /mnt/lvdata/laservision/data/focusvision
	sudo cp -avrf $(AGSCFGDIR)/visionparameters.txt /mnt/lvdata/laservision/data/vision
	sudo dos2unix /mnt/lvdata/laservision/data/vision
	sudo cp -avrf $(AGSCFGDIR)/calibration.txt /mnt/lvdata/laservision/data/calib
	sudo dos2unix /mnt/lvdata/laservision/data/calib
	sudo cp -avrf $(AGSCFGDIR)/version.txt /mnt/lvdata/laservision/data/version
	sudo dos2unix /mnt/lvdata/laservision/data/version
	sudo cp -avrf $(AGSCFGDIR)/information.txt /mnt/lvdata/laservision/data/info
	sudo dos2unix /mnt/lvdata/laservision/data/info
	sudo cp -avrf $(AGSCFGDIR)/initialization.txt /mnt/lvdata/laservision/data/init
	sudo dos2unix /mnt/lvdata/laservision/data/init
	sudo cp -avrf $(AGSCFGDIR)/polarizer.txt /mnt/lvdata/laservision/data/polarizer
	sudo dos2unix /mnt/lvdata/laservision/data/polarizer
	sudo echo "\n" > hobbs
	sudo mv hobbs /mnt/lvdata/laservision/data
	sudo chmod -R 777 /mnt/lvdata/*
	sudo mount -o loop,ro $(BUILDROOTDIR)/output/images/rootfs.ext2 $(BUILDROOTDIR)/output/ext2
	sudo cp -avrf $(BUILDROOTDIR)/output/ext2/* /mnt/lvboot
	sudo cp $(BUILDROOTDIR)/output/images/bzImage /mnt/lvboot
	sudo cp $(AGSCFGDIR)/extlinux.conf /mnt/lvboot
	sync
	sudo umount /dev/sdb1
	sudo umount /dev/sdb2
	sudo umount $(BUILDROOTDIR)/output/ext2
	sudo rm -rf /mnt/lvdata
	sudo rm -rf /mnt/lvboot
	sudo umount /dev/sdb2
