/*
 * crc_calculator.c
 *
 *  Created on: 2015-6-12
 *      Author: Administrator
 */

static unsigned short CRC16_Table[256]=
{
		 0x0000, 0x3d65, 0x7aca, 0x47af, 0xf594, 0xc8f1, 0x8f5e, 0xb23b, 0xd64d, 0xeb28, 0xac87, 0x91e2, 0x23d9, 0x1ebc, 0x5913, 0x6476,
		 0x91ff, 0xac9a, 0xeb35, 0xd650, 0x646b, 0x590e, 0x1ea1, 0x23c4, 0x47b2, 0x7ad7, 0x3d78, 0x1d, 0xb226, 0x8f43, 0xc8ec, 0xf589,
		 0x1e9b, 0x23fe, 0x6451, 0x5934, 0xeb0f, 0xd66a, 0x91c5, 0xaca0, 0xc8d6, 0xf5b3, 0xb21c, 0x8f79, 0x3d42, 0x27, 0x4788, 0x7aed,
		 0x8f64, 0xb201, 0xf5ae, 0xc8cb, 0x7af0, 0x4795, 0x3a, 0x3d5f, 0x5929, 0x644c, 0x23e3, 0x1e86, 0xacbd, 0x91d8, 0xd677, 0xeb12,
		 0x3d36, 0x53, 0x47fc, 0x7a99, 0xc8a2, 0xf5c7, 0xb268, 0x8f0d, 0xeb7b, 0xd61e, 0x91b1, 0xacd4, 0x1eef, 0x238a, 0x6425, 0x5940,
		 0xacc9, 0x91ac, 0xd603, 0xeb66, 0x595d, 0x6438, 0x2397, 0x1ef2, 0x7a84, 0x47e1, 0x4e, 0x3d2b, 0x8f10, 0xb275, 0xf5da, 0xc8bf,
		 0x23ad, 0x1ec8, 0x5967, 0x6402, 0xd639, 0xeb5c, 0xacf3, 0x9196, 0xf5e0, 0xc885, 0x8f2a, 0xb24f, 0x74, 0x3d11, 0x7abe, 0x47db,
		 0xb252, 0x8f37, 0xc898, 0xf5fd, 0x47c6, 0x7aa3, 0x3d0c, 0x69, 0x641f, 0x597a, 0x1ed5, 0x23b0, 0x918b, 0xacee, 0xeb41, 0xd624,
		 0x7a6c, 0x4709, 0xa6, 0x3dc3, 0x8ff8, 0xb29d, 0xf532, 0xc857, 0xac21, 0x9144, 0xd6eb, 0xeb8e, 0x59b5, 0x64d0, 0x237f, 0x1e1a,
		 0xeb93, 0xd6f6, 0x9159, 0xac3c, 0x1e07, 0x2362, 0x64cd, 0x59a8, 0x3dde, 0xbb, 0x4714, 0x7a71, 0xc84a, 0xf52f, 0xb280, 0x8fe5,
		 0x64f7, 0x5992, 0x1e3d, 0x2358, 0x9163, 0xac06, 0xeba9, 0xd6cc, 0xb2ba, 0x8fdf, 0xc870, 0xf515, 0x472e, 0x7a4b, 0x3de4, 0x81,
		 0xf508, 0xc86d, 0x8fc2, 0xb2a7, 0x9c, 0x3df9, 0x7a56, 0x4733, 0x2345, 0x1e20, 0x598f, 0x64ea, 0xd6d1, 0xebb4, 0xac1b, 0x917e,
		 0x475a, 0x7a3f, 0x3d90, 0xf5, 0xb2ce, 0x8fab, 0xc804, 0xf561, 0x9117, 0xac72, 0xebdd, 0xd6b8, 0x6483, 0x59e6, 0x1e49, 0x232c,
		 0xd6a5, 0xebc0, 0xac6f, 0x910a, 0x2331, 0x1e54, 0x59fb, 0x649e, 0xe8, 0x3d8d, 0x7a22, 0x4747, 0xf57c, 0xc819, 0x8fb6, 0xb2d3,
		 0x59c1, 0x64a4, 0x230b, 0x1e6e, 0xac55, 0x9130, 0xd69f, 0xebfa, 0x8f8c, 0xb2e9, 0xf546, 0xc823, 0x7a18, 0x477d, 0xd2, 0x3db7,
		 0xc83e, 0xf55b, 0xb2f4, 0x8f91, 0x3daa, 0xcf, 0x4760, 0x7a05, 0x1e73, 0x2316, 0x64b9, 0x59dc, 0xebe7, 0xd682, 0x912d, 0xac48,
};

/*
 * CRC16���������
 */
unsigned short Cal_CRC16_ByByte(void *p, unsigned int cnt)
{
    unsigned short crc = 0;
    unsigned char  da;
    unsigned char * ptr = (unsigned char * )p;
    while (cnt--)
    {
        da = crc >> 8;  // CRC(h)
        crc <<= 8;
        crc ^= CRC16_Table[da ^ *ptr++];
    }
    return (~crc);
}

/*
 * CRC16 calculate (int start, char *p, int n)
 */
unsigned short Cal_CRC16_By_HalfByte(unsigned char *ptr, int len)
{
	unsigned short crc;
		unsigned char da;
		unsigned short static crc_ta[16]={
		0x0000, 0x3d65, 0x7aca, 0x47af, 0xf594, 0xc8f1, 0x8f5e, 0xb23b,
		0xd64d, 0xeb28, 0xac87, 0x91e2, 0x23d9, 0x1ebc, 0x5913, 0x6476
		};
		crc = 0;
		while(len--)
		{
			da    = crc >> 12; //crc high byte high 4 bits

			crc <<= 4;

			crc  ^= crc_ta[da^(*ptr >> 4)]; //nor data high 4 bits

			da    = crc >> 12; ////crc high byte low 4 bits

			crc <<= 4;

			crc ^= crc_ta[da^(*ptr & 0x0f)];

			ptr++;
		}

	return(crc);
}
