ARGS=--all

default: help

# root of the project (makefile directory)
export ROOT_PATH=$(CURDIR)/

# dependecy install/build directory (rpclib, gtest, boost)
export INSTALLATION_DIR=$(ROOT_PATH)Build/

help:
	@type "${CARLA_BUILD_TOOLS_FOLDER}\Windows.mk.help"

# use PHONY to force next line as command and avoid conflict with folders of the same name
.PHONY: import
import: server
	@"${CARLA_BUILD_TOOLS_FOLDER}/Import.py"

CarlaUE4Editor: LibCarla
	@"${CARLA_BUILD_TOOLS_FOLDER}/BuildCarlaUE4.bat" --build

launch: LibCarla
	@"${CARLA_BUILD_TOOLS_FOLDER}/BuildCarlaUE4.bat" --launch

launch-only:
	@"${CARLA_BUILD_TOOLS_FOLDER}/BuildCarlaUE4.bat" --launch

package: PythonAPI
	@"${CARLA_BUILD_TOOLS_FOLDER}/Package.bat" --ue-version 4.22

docs:
	@doxygen
	@echo "Documentation index at ./Doxygen/html/index.html"

clean:
	@"${CARLA_BUILD_TOOLS_FOLDER}/Package.bat" --clean --ue-version 4.22
	@"${CARLA_BUILD_TOOLS_FOLDER}/BuildCarlaUE4.bat" --clean

	@"${CARLA_BUILD_TOOLS_FOLDER}/BuildPythonAPI.bat" --clean
	@"${CARLA_BUILD_TOOLS_FOLDER}/BuildLibCarla.bat" --clean

rebuild: setup
	@"${CARLA_BUILD_TOOLS_FOLDER}/BuildCarlaUE4.bat" --clean
	@"${CARLA_BUILD_TOOLS_FOLDER}/BuildLibCarla.bat" --rebuild
	@"${CARLA_BUILD_TOOLS_FOLDER}/BuildPythonAPI.bat" --rebuild

check: PythonAPI
	@echo "Not implemented!"

benchmark: LibCarla
	@echo "Not implemented!"

.PHONY: PythonAPI
PythonAPI: LibCarla
	@"${CARLA_BUILD_TOOLS_FOLDER}/BuildPythonAPI.bat" --py3

server: setup
	@"${CARLA_BUILD_TOOLS_FOLDER}/BuildLibCarla.bat" --server

client: setup
	@"${CARLA_BUILD_TOOLS_FOLDER}/BuildLibCarla.bat" --client

.PHONY: LibCarla
LibCarla: setup
	@"${CARLA_BUILD_TOOLS_FOLDER}/BuildLibCarla.bat" --server --client

setup:
	@"${CARLA_BUILD_TOOLS_FOLDER}/Setup.bat" --boost-toolset msvc-14.1

deploy:
	@"${CARLA_BUILD_TOOLS_FOLDER}/Deploy.bat" $(ARGS)
