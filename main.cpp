
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <vector>

template <typename PixelType>
void FloodFill(
	const PixelType* pImage, int imageLineStride,
	uint8_t* pFlags, int flagsLineStride,
	uint16_t x, uint16_t y,
	uint16_t minX, uint16_t maxX,
	uint16_t minY, uint16_t maxY,
	PixelType threshold
	)
{
	std::vector<uint32_t> q(1024*128);
	size_t pos = 1;
	q[0] = x | (y << 16);
	while (pos--) {
		uint32_t xy = q[pos];
		uint32_t x = xy & 0xFFFF;
		uint32_t y = xy >> 16;
		auto pixel = pImage[y * imageLineStride + x];
		auto* flag = &pFlags[y * flagsLineStride + x];
		if (!*flag && pixel >= threshold) {
			*flag = 1;
			q[pos++] = (x - 1) | (y << 16);
			q[pos++] = (x + 1) | (y << 16);
			q[pos++] = x | ((y - 1) << 16);
			q[pos++] = x | ((y + 1) << 16);
		}
	}
}

int main(int argc, char* argv[])
{
	const size_t WIDTH = 640;
	const size_t HEIGHT = 480;
	std::vector<uint8_t> src(WIDTH*HEIGHT);
	uint8_t* pSrc = &src[0];
	
	FILE* f = fopen("test.raw", "rb");
	fread(pSrc, 1, WIDTH*HEIGHT, f);
	fclose(f);
	
	std::vector<uint8_t> flags(WIDTH*HEIGHT);
	uint8_t* pFlags = &flags[0];

	FloodFill(
		pSrc, WIDTH,
		pFlags, WIDTH,
		WIDTH/2, HEIGHT/2,
		10, WIDTH-10,
		10, HEIGHT-10,
		(uint8_t)150
		);
	
	return 0;
}

