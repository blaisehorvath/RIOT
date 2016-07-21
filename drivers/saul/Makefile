SRC = saul_str.c

ifneq (,$(filter saul_gpio,$(USEMODULE)))
  SRC += gpio_saul.c
endif
ifneq (,$(filter saul_bme280,$(USEMODULE)))
  SRC += saul_bme280.c
  $(warning lofasz)
endif

include $(RIOTBASE)/Makefile.base
