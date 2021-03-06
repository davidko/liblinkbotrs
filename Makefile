VERSION = $(shell grep 'version' Cargo.toml | sed 's/version *= *"\([0-9\.]*\)"/\1/g')

all: linkbotrs

cpp_test: 
	g++ -g -std=c++11 -Iinclude c_test/test.cpp -c -o c_test/test.o
	g++ -L./target/debug c_test/test.o -llinkbotrs -o c_test/test
	LD_LIBRARY_PATH=./target/debug c_test/test

linkbotrs:
	cargo build --release -j8

zip:
	mkdir -p _staging/usr/lib
	mkdir -p _staging/usr/include
	cp target/release/liblinkbotrs.so _staging/usr/lib
	cp include/* _staging/usr/include
	tar czf dist/linkbotrs-$(VERSION).tar.gz -C _staging usr 
	rm -rf _staging
