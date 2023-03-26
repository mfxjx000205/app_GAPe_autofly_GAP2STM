io=uart
PMSIS_OS = freertos

APP = communicate
APP_SRCS += communicate.c ../../../lib/cpx/src/com.c ../../../lib/cpx/src/cpx.c
APP_INC=../../../lib/cpx/inc
APP_CFLAGS += -O3 -g
APP_CFLAGS += -DconfigUSE_TIMERS=1 -DINCLUDE_xTimerPendFunctionCall=1

include $(RULES_DIR)/pmsis_rules.mk
