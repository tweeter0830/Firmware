#include <libtomcrypt/tomcrypt_custom.h>
#include <libtomcrypt/tomcrypt.h>

#include <systemlib/err.h>

#define PACKET_SIZE  5
#define KEY_LEN 16
#define IV_LEN 12
#define AAD_LEN 6

__EXPORT int gcm_main(void);

int test(unsigned char *pt, unsigned long ptlen, unsigned char *iv,
		unsigned long ivlen, unsigned char *aad, unsigned long aadlen,
		gcm_state *gcm, gcm_state *gcm_2);

int test(unsigned char *pt, unsigned long ptlen, unsigned char *iv,
		unsigned long ivlen, unsigned char *aad, unsigned long aadlen,
		gcm_state *gcm, gcm_state *gcm_2) {
	int err;
	unsigned long taglen;
	unsigned char tag[16] = {0};
	unsigned long taglen_2;
	unsigned char tag2[16] = {0};
	unsigned char ct[PACKET_SIZE] = {0};
	unsigned char pt_2[PACKET_SIZE] = {0};
	/* reset the state */
	if ((err = gcm_reset(gcm)) != CRYPT_OK) {
		return err;
	}
	/* Add the IV */
	if ((err = gcm_add_iv(gcm, iv, ivlen)) != CRYPT_OK) {
		return err;
	}
	/* Add the AAD (note: aad can be NULL if aadlen == 0) */
	if ((err = gcm_add_aad(gcm, aad, aadlen)) != CRYPT_OK) {
		return err;

	}
	/* process the plaintext */
	if ((err = gcm_process(gcm, pt, ptlen, ct, GCM_ENCRYPT)) != CRYPT_OK) {
		return err;
	}
	/* Finish up and get the MAC tag */
	taglen = sizeof(tag);
	if ((err = gcm_done(gcm, tag, &taglen)) != CRYPT_OK) {
		return err;
	}

	//messing around
	//aad[0] = 45;
	//iv[0] = 1;
	//ct[1] = 38;
	if ((err = gcm_reset(gcm_2)) != CRYPT_OK) {
		return err;
	}
	/* Add the IV */
	if ((err = gcm_add_iv(gcm_2, iv, ivlen)) != CRYPT_OK) {
		return err;
	}
	/* Add the AAD (note: aad can be NULL if aadlen == 0) */
	if ((err = gcm_add_aad(gcm_2, aad, aadlen)) != CRYPT_OK) {
		return err;

	}
	/* process the plaintext */
	if ((err = gcm_process(gcm_2, pt_2, ptlen, ct, GCM_DECRYPT)) != CRYPT_OK) {
		return err;
	}
	/* Finish up and get the MAC tag */
	taglen = sizeof(tag);
	if ((err = gcm_done(gcm_2, tag2, &taglen_2)) != CRYPT_OK) {
		return err;
	}

	if (XMEMCMP(tag, tag2, 16)) {

		printf("\nTag on ciphertext wrong :\n");
		warnx("tag #1: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
		              tag[0], tag[1], tag[2], tag[3], tag[4], tag[5], tag[6], tag[7], tag[8], tag[9], tag[10], tag[11], tag[12], tag[13], tag[14], tag[15]);
		warnx("tag #2: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
		              tag2[0], tag2[1], tag2[2], tag2[3], tag2[4], tag2[5], tag2[6], tag2[7], tag2[8], tag2[9], tag2[10], tag2[11], tag2[12], tag2[13], tag2[14], tag2[15]);
	}
	else {
		printf("\nTest OK!\n");
	}

	return CRYPT_OK;
}
int gcm_main(void) {
	gcm_state gcm, gcm_2;

	unsigned char key[KEY_LEN] = { 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2,
			0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c };
	unsigned char IV[IV_LEN] = {0};

	unsigned char pt[PACKET_SIZE] = { 0x2b, 0x2b, 0x2b, 0x2b, 0x2b };
	int err, x;
	unsigned long ptlen = PACKET_SIZE;


	const unsigned char aad[AAD_LEN] = { 0x25, 0x34, 0xFF, 0x00, 0xCC, 0xAB};
	unsigned long aadlen = AAD_LEN;



	/* register AES */
	register_cipher(&aes_desc);
	/* init the GCM state */
	if ((err = gcm_init(&gcm, find_cipher("aes"), key, 16)) != CRYPT_OK) {
		puts("Error");
	}

	/* init the GCM state */
	if ((err = gcm_init(&gcm_2, find_cipher("aes"), key, 16)) != CRYPT_OK) {
		puts("Error");
	}

	/* handle us some packets */
	//for (;;) {
	/* use IV as counter (12 byte counter) */
	for (x = 11; x >= 0; x--) {
		if (++IV[x]) {
			break;
		}
	}

	warnx("IV: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
		              IV[0], IV[1], IV[2], IV[3], IV[4], IV[5], IV[6], IV[7], IV[8], IV[9], IV[10], IV[11]);

	if ((err = test(pt, ptlen, IV, IV_LEN, aad, aadlen, &gcm, &gcm_2))
			!= CRYPT_OK) {
		puts("Error");
	}
//}

	///gcm_test();

	return EXIT_SUCCESS;
}

