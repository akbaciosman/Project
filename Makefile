all: piMotor

#XENO_CONFIG = /usr/xenomai/bin/xeno-config
#CFLAGS = $(shell $(XENO_CONFIG) --skin=native --cflags)
#LDFLAGS = $(shell $(XENO_CONFIG) --skin=native --ldflags)

HDRS = helper_3dmath.h I2Cdev.h MPU6050_6Axis_MotionApps20.h MPU6050.h
CMN_OBJS = I2Cdev.o MPU6050.o
RAW_OBJS = piMotor.o

#CC=$(shell $(XENO_CONFIG) --cc)

CC = g++

# Set DMP FIFO rate to 20Hz to avoid overflows on 3d demo
# MPU6050_6Axis_MotionApps20.h for details.

#CXXFLAGS = -DDMP_FIFO_RATE=9 -Wall -g -O2 `pkg-config gtkmm-3.0 --cfla$

$(CMN_OBJS) $(DMP_OBJS) $(RAW_OBJS) : $(HDRS)

piMotor: $(CMN_OBJS) $(RAW_OBJS)
	$(CC) -o $@ $^ -lm -lpigpio -lrt
	#$(CFLAGS) $(LDFLAGS)

clean:
	rm -f $(CMN_OBJS) $(DMP_OBJS) $(D3D_OBJS) $(RAW_OBJS) demo_raw
