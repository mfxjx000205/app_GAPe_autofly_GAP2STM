io=uart
PMSIS_OS = freertos

APP = communicate

APP_SRCS += communicate.c ../../../lib/cpx/src/com.c ../../../lib/cpx/src/cpx.c 
#APP_SRCS += ../../../lib/tinyoctomap/src/octoMap.c ../../../lib/tinyoctomap/src/octoTree.c ../../../lib/tinyoctomap/src/octoNode.c ../../../lib/tinyoctomap/src/octoNodeSet.c
APP_SRCS += auxiliary_tool.c circularQueue.c compute_tool.c coordinateQueue.c octoMap.c octoNode.c octoNodeSet.c octoTree.c
APP_INC = ../../../lib/cpx/inc ../../../lib/tinyoctomap/inc

APP_CFLAGS += -O3 -g
APP_CFLAGS += -DconfigUSE_TIMERS=1 -DINCLUDE_xTimerPendFunctionCall=1

include $(RULES_DIR)/pmsis_rules.mk
