default: help

help:
	@less ${CARLA_BUILD_TOOLS_FOLDER}/Linux.mk.help

build: LibCarla.server.release
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildCarlaUE4.sh --build $(ARGS)

launch: LibCarla.server.release
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildCarlaUE4.sh --build --launch $(ARGS)

launch-only:
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildCarlaUE4.sh --launch $(ARGS)

import: CarlaUE4Editor PythonAPI build.utils
	@${CARLA_BUILD_TOOLS_FOLDER}/Import.py $(ARGS)

package: CarlaUE4Editor PythonAPI
	@${CARLA_BUILD_TOOLS_FOLDER}/Package.sh $(ARGS)

docs:
	@doxygen
	@echo "Documentation index at ./Doxygen/html/index.html"

clean.LibCarla:
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildLibCarla.sh --clean
clean.PythonAPI:
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildPythonAPI.sh --clean
clean.CarlaUE4Editor:
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildCarlaUE4.sh --clean
clean: clean.CarlaUE4Editor clean.PythonAPI clean.LibCarla

rebuild: setup
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildLibCarla.sh --rebuild
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildPythonAPI.sh --rebuild
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildCarlaUE4.sh --rebuild

hard-clean:
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildCarlaUE4.sh --hard-clean
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildPythonAPI.sh --clean
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildLibCarla.sh --clean
	@echo "To force recompiling dependencies run: rm -Rf ${CARLA_BUILD_FOLDER}"

check: LibCarla PythonAPI
	@${CARLA_BUILD_TOOLS_FOLDER}/Check.sh --all $(ARGS)

check.LibCarla: LibCarla
	@${CARLA_BUILD_TOOLS_FOLDER}/Check.sh --libcarla-debug --libcarla-release $(ARGS)

check.LibCarla.debug: LibCarla.debug
	@${CARLA_BUILD_TOOLS_FOLDER}/Check.sh --libcarla-debug $(ARGS)

check.LibCarla.release: LibCarla.release
	@${CARLA_BUILD_TOOLS_FOLDER}/Check.sh --libcarla-release $(ARGS)

check.PythonAPI: PythonAPI
	@${CARLA_BUILD_TOOLS_FOLDER}/Check.sh --python-api-3 $(ARGS)

check.PythonAPI.2: PythonAPI.2
	@${CARLA_BUILD_TOOLS_FOLDER}/Check.sh --python-api-2 $(ARGS)

check.PythonAPI.3: PythonAPI.3
	@${CARLA_BUILD_TOOLS_FOLDER}/Check.sh --python-api-3 $(ARGS)

benchmark: LibCarla.release
	@${CARLA_BUILD_TOOLS_FOLDER}/Check.sh --benchmark $(ARGS)
	@cat profiler.csv

smoke_tests:
	@${CARLA_BUILD_TOOLS_FOLDER}/Check.sh --smoke-2 --smoke-3 $(ARGS)

examples:
	@for D in ${CARLA_EXAMPLES_FOLDER}/*; do [ -d "$${D}" ] && make -C $${D} build; done

run-examples:
	@for D in ${CARLA_EXAMPLES_FOLDER}/*; do [ -d "$${D}" ] && make -C $${D} run.only; done

CarlaUE4Editor: LibCarla.server.release
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildCarlaUE4.sh --build

.PHONY: PythonAPI
PythonAPI: LibCarla.client.release
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildPythonAPI.sh --py3

PythonAPI.2: LibCarla.client.release
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildPythonAPI.sh --py2

PythonAPI.3: LibCarla.client.release
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildPythonAPI.sh --py3

PythonAPI.rss: LibCarla.client.rss.release
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildPythonAPI.sh --py3 --rss

PythonAPI.rss.2: LibCarla.client.rss.release
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildPythonAPI.sh --py2 --rss

PythonAPI.rss.3: LibCarla.client.rss.release
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildPythonAPI.sh --py3 --rss

PythonAPI.docs:
	@python PythonAPI/docs/doc_gen.py
	@cd PythonAPI/docs && python3 bp_doc_gen.py

.PHONY: LibCarla
LibCarla: LibCarla.release LibCarla.debug

LibCarla.debug: LibCarla.server.debug LibCarla.client.debug
LibCarla.release: LibCarla.server.release LibCarla.client.release

LibCarla.server: LibCarla.server.debug LibCarla.server.release
LibCarla.server.debug: setup
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildLibCarla.sh --server --debug
LibCarla.server.release: setup
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildLibCarla.sh --server --release

LibCarla.client: LibCarla.client.debug LibCarla.client.release
LibCarla.client.debug: setup
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildLibCarla.sh --client --debug
LibCarla.client.release: setup
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildLibCarla.sh --client --release

LibCarla.client.rss: LibCarla.client.rss.debug LibCarla.client.rss.release
LibCarla.client.rss.debug: setup ad-rss
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildLibCarla.sh --client --debug --rss
LibCarla.client.rss.release: setup ad-rss
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildLibCarla.sh --client --release --rss

setup:
	@${CARLA_BUILD_TOOLS_FOLDER}/Setup.sh

ad-rss:
	@${CARLA_BUILD_TOOLS_FOLDER}/Ad-rss.sh

deploy:
	@${CARLA_BUILD_TOOLS_FOLDER}/Deploy.sh $(ARGS)

pretty:
	@${CARLA_BUILD_TOOLS_FOLDER}/Prettify.sh $(ARGS)

build.utils: setup
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildUtilsDocker.sh
