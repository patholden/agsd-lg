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
	rm -f *.o agsd $(BUILDROOTFSDIR)/etc
	rm -rf $(BUILDROOTFSDIR)/debug
	rm -rf $(BUILDROOTTGTDIR)/etc/ags
usb_install:
	cp $(AGSCFGDIR)/autofocus.txt $(BUILDROOTFSDIR)/lv/data/autofocus
	dos2unix $(BUILDROOTFSDIR)/lv/data/autofocus
	cp $(AGSCFGDIR)/visionfocus.txt $(BUILDROOTFSDIR)/lv/data/focusvision
	dos2unix $(BUILDROOTFSDIR)/lv/data/focusvision
	cp $(AGSCFGDIR)/visionparameters.txt $(BUILDROOTFSDIR)/lv/data/vision
	dos2unix $(BUILDROOTFSDIR)/lv/data/vision
	cp $(AGSCFGDIR)/calibration.txt $(BUILDROOTFSDIR)/lv/data/calib
	dos2unix $(BUILDROOTFSDIR)/lv/data/calib
	cp $(AGSCFGDIR)/version.txt $(BUILDROOTFSDIR)/lv/data/version
	dos2unix $(BUILDROOTFSDIR)/lv/data/version
	cp $(AGSCFGDIR)/information.txt $(BUILDROOTFSDIR)/lv/data/info
	dos2unix $(BUILDROOTFSDIR)/lv/data/info
	cp $(AGSCFGDIR)/initialization.txt $(BUILDROOTFSDIR)/lv/data/init
	dos2unix $(BUILDROOTFSDIR)/lv/data/init
	cp $(AGSCFGDIR)/polarizer.txt $(BUILDROOTFSDIR)/lv/data/polarizer
	dos2unix $(BUILDROOTFSDIR)/lv/data/polarizer
	echo "\0" > $(BUILDROOTFSDIR)/lv/data/hobbs
	chmod +w $(BUILDROOTFSDIR)/lv/data/*
	chmod 777 agsd
	cp agsd $(BUILDROOTFSDIR)/lv/sbin
	chmod 777 $(AGSCFGDIR)/S95agsd
	cp $(AGSCFGDIR)/S95agsd $(BUILDROOTTGTDIR)/etc/init.d
	chmod 777 $(AGSCFGDIR)/S50sshd
	cp $(AGSCFGDIR)/S50sshd $(BUILDROOTTGTDIR)/etc/init.d
	cp $(AGSCFGDIR)/sshd_config $(BUILDROOTTGTDIR)/etc/ssh
	cp $(AGSCFGDIR)/usb_fstab $(BUILDROOTTGTDIR)/etc/fstab
	cp $(AGSCFGDIR)/gdbinit $(BUILDROOTFSDIR)/.gdbinit
	cp $(AGSCFGDIR)/skeleton.mk $(BUILDROOTDIR)/package/skeleton/
	cp $(AGSCFGDIR)/ags-busybox-config $(BUILDROOTDIR)/package/busybox
	cp $(AGSCFGDIR)/ags-buildroot-config $(BUILDROOTDIR)/.config
mmc_install:
	cp $(AGSCFGDIR)/autofocus.txt $(BUILDROOTFSDIR)/lv/data/autofocus
	dos2unix $(BUILDROOTFSDIR)/lv/data/autofocus
	cp $(AGSCFGDIR)/visionfocus.txt $(BUILDROOTFSDIR)/lv/data/focusvision
	dos2unix $(BUILDROOTFSDIR)/lv/data/focusvision
	cp $(AGSCFGDIR)/visionparameters.txt $(BUILDROOTFSDIR)/lv/data/vision
	dos2unix $(BUILDROOTFSDIR)/lv/data/vision
	cp $(AGSCFGDIR)/calibration.txt $(BUILDROOTFSDIR)/lv/data/calib
	dos2unix $(BUILDROOTFSDIR)/lv/data/calib
	cp $(AGSCFGDIR)/version.txt $(BUILDROOTFSDIR)/lv/data/version
	dos2unix $(BUILDROOTFSDIR)/lv/data/version
	cp $(AGSCFGDIR)/information.txt $(BUILDROOTFSDIR)/lv/data/info
	dos2unix $(BUILDROOTFSDIR)/lv/data/info
	cp $(AGSCFGDIR)/initialization.txt $(BUILDROOTFSDIR)/lv/data/init
	dos2unix $(BUILDROOTFSDIR)/lv/data/init
	cp $(AGSCFGDIR)/polarizer.txt $(BUILDROOTFSDIR)/lv/data/polarizer
	dos2unix $(BUILDROOTFSDIR)/lv/data/polarizer
	touch $(BUILDROOTFSDIR)/lv/data/hobbs
	chmod +w $(BUILDROOTFSDIR)/lv/data/*
	chmod 777 agsd
	cp agsd $(BUILDROOTFSDIR)/lv/sbin
	chmod 777 $(AGSCFGDIR)/S95agsd
	cp $(AGSCFGDIR)/S95agsd $(BUILDROOTTGTDIR)/etc/init.d
	chmod 777 $(AGSCFGDIR)/S50sshd
	cp $(AGSCFGDIR)/S50sshd $(BUILDROOTTGTDIR)/etc/init.d
	cp $(AGSCFGDIR)/sshd_config $(BUILDROOTTGTDIR)/etc/ssh
	cp $(AGSCFGDIR)/mmc_fstab $(BUILDROOTTGTDIR)/etc/fstab
	cp $(AGSCFGDIR)/gdbinit $(BUILDROOTFSDIR)/.gdbinit
	cp $(AGSCFGDIR)/skeleton.mk $(BUILDROOTDIR)/package/skeleton/
	cp $(AGSCFGDIR)/ags-busybox-config $(BUILDROOTDIR)/package/busybox
	cp $(AGSCFGDIR)/ags-buildroot-config $(BUILDROOTDIR)/.config

burnusb:
	sudo umount /dev/sdb1
	sudo mount /dev/sdb1 /mnt/stick
	sudo mount -o loop,ro $(BUILDROOTDIR)/output/images/rootfs.ext2 $(BUILDROOTDIR)/output/ext2
	sudo cp -avrf $(BUILDROOTDIR)/output/ext2/* /mnt/stick
	sudo cp $(BUILDROOTDIR)/output/images/bzImage /mnt/stick
	sudo cp $(AGSCFGDIR)/extlinux.conf /mnt/stick
	sudo umount /dev/sdb1
	sudo umount $(BUILDROOTDIR)/output/ext2
burnflash:
	sudo umount /dev/sdb1
	sudo mount /dev/sdb1 /mnt/stick
	sudo mount -o loop,ro $(BUILDROOTDIR)/output/images/rootfs.ext2 $(BUILDROOTDIR)/output/ext2
	sudo cp -avrf $(BUILDROOTDIR)/output/ext2/* /mnt/stick
	sudo cp $(BUILDROOTDIR)/output/images/bzImage /mnt/stick
	sudo cp $(AGSCFGDIR)/mmc_extlinux.conf /mnt/stick/extlinux.conf
	sudo chmod -R 777 /mnt/stick/lv/data
	sudo chmod -R 777 /mnt/stick/usr/sbin/agsd
	sudo umount /dev/sdb1
	sudo umount $(BUILDROOTDIR)/output/ext2
