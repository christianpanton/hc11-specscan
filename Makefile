CCPATH=/opt/sdcc-3.4.0/bin
FLASHPATH=/home/panton/build/stm8flash

SDCC=$(CCPATH)/sdcc
SDLD=$(CCPATH)/sdld
OBJECT=main.ihx

.PHONY: all clean flash

all: $(OBJECT)

clean:
	rm -f $(OBJECT) *.lk *.map *.lst *.sym *.asm *.rst *.rel *.cdb

flash: $(OBJECT)
	$(FLASHPATH)/stm8flash -cstlinkv2 -pstm8s003 -w $(OBJECT)

%.ihx: %.c
	$(SDCC) -lstm8 -mstm8 --out-fmt-ihx $(CFLAGS) $(LDFLAGS) $<
