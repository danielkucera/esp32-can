typedef union {
	uint8_t U[8]; /**< \brief Unsigned access */
	struct {
    /* byte 0 */
		uint8_t EP1_Zaehler : 4;
		uint8_t EP1_Failure_Sta : 2;
		bool EP1_Sta_EPB : 1;
		bool EP1_Sta_Schalter : 1;
    /* byte 1 */
		uint8_t EP1_Spannkraft : 5;
		uint8_t EP1_Schalterinfo : 2;
		bool EP1_Sta_NWS : 1;
    /* byte 2 */
		uint8_t EP1_Neig_winkel : 8;
    /* byte 3 */
		uint8_t EP1_Verzoegerung : 8;
    /* byte 4 */
		bool EP1_Failureeintr : 1;
		bool EP1_Freigabe_Ver : 1;
		bool EP1_AutoHold_zul : 1;
		bool EP1_AutoHold_active : 1;
		bool EP1_SleepInd : 1;
		bool EP1_Status_Kl_15 : 1;
		bool EP1_Lampe_AutoP : 1;
		bool EP1_Bremslicht : 1;
    /* byte 5 */
		bool EP1_Warnton1 : 1;
		bool EP1_Warnton2 : 1;
		bool EP1_AnfShLock : 1;
		bool EPB_Autoholdlampe : 1;
		bool Unknown : 1;
		bool EP1_KuppModBer : 1;
        bool Unknown2 : 1;
		bool EP1_HydrHalten : 1;
    /* byte 6 */
		bool EP1_Fkt_Lampe : 1;
		bool EP1_Warnton : 1;
		bool EP1_Failure_BKL : 1;
		bool EP1_Failure_gelb : 1;
		uint8_t EP1__Text : 4;
    /* byte 7 */
		uint8_t EP1_Checksum : 8;
	} B;
} mEPB_1;

typedef union {
	uint8_t U[8]; /**< \brief Unsigned access */
	struct {
    /* byte 0 */
    uint16_t BR5_Giergeschw : 14;           // @1+ (0.01,0) [0|100] "Grad/sec" XXX
    bool BR5_Sta_Gierrate : 1;              // @1+ (1,0) [0|1] "" XXX
    bool BR5_Vorzeichen : 1;                // @1+ (1,0) [0|1] "" XXX
    uint16_t BR5_Bremsdruck : 12;           // @1+ (0.1,0) [0|250] "bar" XXX
    bool BR5_Stillstand : 1;                // @1+ (1,0) [0|1] "" XXX
    bool BR5_Druckvalid : 1;                // @1+ (1,0) [0|1] "" XXX
    bool BR5_Sta_Druck : 1;                 // @1+ (1,0) [0|1] "" XXX
    bool BR5_Sign_Druck : 1;                // @1+ (1,0) [0|1] "" XXX
    bool Bit_32 : 1;
    bool Bit_33 : 1;
    bool ESP_Rollenmodus_Deactiveieren : 1; // @1+ (1,0) [0|1] "" XXX
    bool ESP_Stat_FallBack_eBKV : 1;        // @1+ (1,0) [0|1] "" XXX
    uint8_t ESP_Anforderung_EPB : 2;        // @1+ (1,0) [0|3] "" XXX
    bool ESP_Autohold_active : 1;           // @1+ (1,0) [0|1] "" XXX
    bool ESP_Autohold_Standby : 1;          // @1+ (1,0) [0|1] "" XXX
    bool BR5_Anhi_Sta : 1;                  // @1+ (1,0) [0|1] "" XXX
    bool BR5_Anhi_akt : 1;                  // @1+ (1,0) [0|1] "" XXX
    bool BR5_v_Ueberw : 1;                  // @1+ (1,0) [0|1] "" XXX
    bool BR5_Bremslicht : 1;                // @1+ (1,0) [0|1] "" XXX
    bool BR5_Notbremsung : 1;               // @1+ (1,0) [0|1] "" XXX
    bool BR5_Fahrer_tritt_ZBR_Schw : 1;     // @1+ (1,0) [0|1] "" XXX
    bool BR5_AWV2_Bremsruck : 1;            // @1+ (1,0) [0|1] "" XXX
    bool BR5_AWV2_Failure : 1;              // @1+ (1,0) [0|1] "" XXX
    bool BR5_ZT_Rueckk_Umsetz : 1;          // @1+ (1,0) [0|1] "" XXX
    bool BR5_ANB_CM_Rueckk_Umsetz : 1;      // @1+ (1,0) [0|1] "" XXX
    bool BR5_HDC_bereit : 1;                // @1+ (1,0) [0|1] "" XXX
    bool BR5_ECD_Lampe : 1;                 // @1+ (1,0) [0|0] "" XXX
    uint8_t BR5_Zaehler : 4;                // @1+ (1,0) [0|15] "" XXX
    uint8_t BR5_Checksumme : 8;             // @1+ (1,0) [0|0] "" XXX
	} B;
} Bremse_5;

