MAKE_FLAGS?=-j5


clean:
	rm -Rf build

all:
	mkdir -p build;
	cd build; cmake \
					-DHUMOTO_BUILD_TESTS=ON \
					-DHUMOTO_BUILD_REGRESSION_TESTS=ON \
					-DHUMOTO_MODULES_DIRS=`pwd`/../extra_modules/ \
					-DHUMOTO_BRIDGE_qpOASES=ON \
					-DHUMOTO_BRIDGE_config_yaml=ON \
					-DHUMOTO_MODULE_example=ON \
					~/profiles/devel/src/humoto/humoto-core/
	cd build; ${MAKE} ${MAKE_FLAGS}

tests: all
	cd build; ${MAKE} ${MAKE_FLAGS} test

update:
	git submodule update --init

.PHONY: build
