CONTIKI_PROJECT = foo
all: $(CONTIKI_PROJECT)

CONTIKI = contiki
TARGETDIRS += platform
APPDIRS += ${addprefix karl-apps/, $(APPS)}
APPS+=serial-shell shell-rime-ping

include $(CONTIKI)/Makefile.include
