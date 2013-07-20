CONTIKI_PROJECT = foo
all: $(CONTIKI_PROJECT)

CONTIKI = contiki
TARGETDIRS += platform
APPDIRS += ${addprefix karl-apps/, $(APPS)}
APPS+=serial-shell

include $(CONTIKI)/Makefile.include
