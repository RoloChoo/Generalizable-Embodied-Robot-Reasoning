# =========================
# VisualTracking (Webots)
# OpenCV 2.4.13 + C++98
# =========================

CXX_SOURCES = VisualTracking.cpp
CXXFLAGS += -std=c++98

# --- OpenCV 설정 (pkg-config 있으면 자동으로 경로 인식) ---
INCLUDE  += $(shell pkg-config --cflags opencv)
LIBRARIES+= $(shell pkg-config --libs opencv)

# --- DARwIn-OP 매니저 및 프레임워크 연결 ---
RESOURCES_PATH = $(WEBOTS_HOME)/resources/projects/robots/darwin-op
PROTECTED_RESOURCES_PATH = $(subst $(space),\ ,$(strip $(subst \,/,$(RESOURCES_PATH))))
INCLUDE += -I"$(RESOURCES_PATH)/libraries/managers/include" \
           -I"$(RESOURCES_PATH)/libraries/darwin/darwin/Framework/include"
LIBRARIES += -L"$(RESOURCES_PATH)/libraries/darwin" -ldarwin \
             -L"$(RESOURCES_PATH)/libraries/managers" -lmanagers

# --- Webots 글로벌 빌드 시스템 포함 ---
space :=
space +=
WEBOTS_HOME_PATH = $(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
