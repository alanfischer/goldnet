#include "app.h"

int main(int argc, char **argv) {
	shove::Wires udp;
	return shove::run(argc, argv, udp);
}
