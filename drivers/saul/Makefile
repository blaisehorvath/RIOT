SRC = saul_str.c

ifneq (,$(filter saul_gpio,$(USEMODULE)))
  SRC += gpio_saul.c
endif
ifneq (,$(filter saul_bme280,$(USEMODULE)))
  SRC += bme280_saul.c
  SRC += bme280.c
endif

include $(RIOTBASE)/Makefile.base
