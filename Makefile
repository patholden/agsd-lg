#
#        -Wall -W -Wbad-function-cast -Wcast-qual \
#	-Wstrict-prototypes \
#	-Wmissing-prototypes -Wmissing-declarations -Wredundant-decls \
#
#       $Id: Makefile,v 1.30 2000/09/28 19:57:01 ags-sw Exp ags-sw $
#
BUILDDIR = $(HOME)/buildroot
STAGING_DIR = $(BUILDDIR)/output/host/usr/x86_64-buildroot-linux-gnu/sysroot
TOOLDIR = $(BUILDDIR)/output/host/usr/bin/x86_64-buildroot-linux-gnu
CC=$(TOOLDIR)-gcc
LD=$(TOOLDIR)-ld
AS=$(TOOLDIR)-as
AR=$(TOOLDIR)/x86_64-buildroot-linux-uclibc-ar

#
#LDFLAGS = -L=$(LIBDIR) -lc -lm -lncurses --entry main
#LDFLAGS=-Wl,--sysroot=$(STAGING_DIR) -Wl,--verbose=9 -Wl,--error-poison-system-directories -L$(STAGING_DIR)/lib -L$(STAGING_DIR)/usr/lib -lc -lm
LDFLAGS=-Wl,--sysroot=$(STAGING_DIR) -Wl,--error-poison-system-directories -L$(STAGING_DIR)/lib -L$(STAGING_DIR)/usr/lib -lc -lm
#
### EXTRA_CFLAGS = -DDEBUG_READ -DDEBUG_WRITE -DDUMMY_TRANS
## EXTRA_CFLAGS = -DDEBUG_READ -DDEBUG_WRITE
## EXTRA_CFLAGS = -DRDEBUG
# EXTRA_CFLAGS = -DSDEBUG -DDEBUG_READ -DDEBUG_WRITE
## EXTRA_CFLAGS = -DSDEBUG -DFLASHLED -DDEBUG_READ -DDEBUG_WRITE -DCENTROID
### EXTRA_CFLAGS = -DFLASHLED -DCENTROID -DZDEBUG -DDEBUG_READ -DRDEBUG -DSDEBUG -DLOUTDEBUG -DQDEBUG -DANGDEBUG
# Next one is one we were using
#EXTRA_CFLAGS = -DCENTROID -DZDEBUG -DDEBUG_READ -DRDEBUG -DSDEBUG -DLOUTDEBUG -DQDEBUG -DYDEBUG
EXTRA_CFLAGS = -DPATDEBUG -DCDEBUG
### EXTRA_CFLAGS = -DCENTROID -DZDEBUG  -DSDEBUG -DWRITEHEX -DREADHEX -DRDEBUG  -DQDEBUG -DOLDLED -DKHHDEBUG -DKHHDEBUG -DODEBUG -DLS_DEBUG -DGSDEBUG -DGTUDEBUG

# EXTRA_CFLAGS = -DCENTROID -DZDEBUG  -DSDEBUG -DWRITEHEX -DREADHEX -DRDEBUG  -DQDEBUG -DOLDLED -DKHHDEBUG -DKHHDEBUG -DODEBUG -DGSDEBUG  -DLG_DEBUG  -DZZZDEBUG

### EXTRA_CFLAGS = -DCENTROID -DWRITEHEX -DREADHEX -DZZZDEBUG

## CFLAGS = -pg -g -DLASER_DEFINED -DDEBUG -m486
#CFLAGS = -g -DLASER_DEFINED -march=atom -Wall -Wextra -Werror
USERINCLUDE    := \
	-I./ \
	-I../../linux_headers

CFLAGS = -g -DLASER_DEFINED -march=atom  -Wall $(USERINCLUDE)
# CFLAGS = -g -DLASER_DEFINED -ICIncludes
# CFLAGS = -DLASER_DEFINED -ICIncludes 
#
AGS_OBJECTS = L3DTransform.o \
	      AngleCorrections.o \
              APTParser.o \
              CRCHandler.o  \
              comm_loop.o  \
              parse_data.o  \
	      DoLevelScan.o \
	      DoAutoFocusCmd.o \
              LaserCmds.o \
	      LaserPattern.o \
	      LaserInterface.o \
              BoardComm.o Events.o \
              SensorRegistration.o \
              3DTransform.o \
	      FullRegManager.o \
	      SearchStopHandler.o \
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
	      CalculateTransform.o
#
#
agsd : compile.h Main.o $(AGS_OBJECTS) SensorSearch.o Files.o
	$(CC) $(LDFLAGS) -o $@ $(AGS_OBJECTS) Main.o SensorSearch.o Files.o
	mv -f compile.h old.compile.h.old

ISensorSearch.o : ISensorSearch.c
	$(CC) ISensorSearch.c -o ISensorSearch.o -c $(CFLAGS)

jigqc : $(JIG_OBJECTS) jigqc.o
	$(CC) jigqc.o -o jigqc ${JIG_OBJECTS} -g -lm /usr/lib/libncurses.a -static

oneside : $(AGS_OBJECTS) oneside.o
	$(CC) oneside.o -o oneside ${AGS_OBJECTS} fork.CTB.o SensorSearch.o -g -lm /usr/lib/libncurses.a -static

%.o: %.c
	$(CC) -c -o $@ $(CFLAGS) $(EXTRA_CFLAGS) $<

Main.o: compile.h asciiID.h Main.c
	$(CC) -c -o Main.o $(CFLAGS) $(EXTRA_CFLAGS) Main.c

Files.o: compile.h Files.c
	$(CC) -c -o Files.o $(CFLAGS) $(EXTRA_CFLAGS) Files.c

jigqc.o: jigqc.c
	$(CC) -c -o jigqc.o $(CFLAGS) $(EXTRA_CFLAGS) jigqc.c

oneside.o: oneside.c
	$(CC) -c -o oneside.o $(CFLAGS) $(EXTRA_CFLAGS) oneside.c

compile.h:
	@echo \#define AGS_COMPILE_TIME \"`date`\" >> .ver
	@echo \#define AGS_UNIX_TIME \"`date +%s`\" >> .ver
	@echo \#define AGS_COMPILE_BY \"`whoami`\" >> .ver
	@echo \#define AGS_COMPILE_HOST \"assemblyguide.com\" >> .ver
	@mv -f .ver $@

asciiID.h:
#	if [ ! -e major ] ; then (echo 4 > major) ; fi
#	echo "scale=3 ; 0.001 + " "0`cat minor`" | bc > minor
#	echo "scale=3 ; " `cat major` " + " `cat minor` | bc > majorminor
	echo  > asciiID.h
	echo  >> asciiID.h
	echo  \#define AGS_ASCII_ID \\ >> asciiID.h
	echo -n "          " >> asciiID.h
	echo -n \"laserguide >> asciiID.h
	echo -n " " >> asciiID.h
	echo -n `date +%Y` >> asciiID.h
	echo -n "  " >> asciiID.h
	echo -n version >> asciiID.h
	echo -n " " >> asciiID.h
	echo -n `cat majorminor`\" >> asciiID.h
	echo >> asciiID.h



oclean:
	rm -f *.o agsd ags.gz

clean:
	rm -f *.o agsd nof old.compile.h.old Iags ags.gz asciiID.h
install:
	chmod 755 agsd
	cp agsd /home/patti/buildroot/board/ags/rootfs_overlay/agsd
