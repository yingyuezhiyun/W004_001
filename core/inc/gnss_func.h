#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "third_party/minmea.h"
    typedef enum
    {
        GNSS_DATA_NMEA,
        GNSS_DATA_RAW,
    } gnss_DataType_t;

    typedef struct
    {
        int fd;
        gnss_DataType_t data_type;
        char Nmea_sentence[MINMEA_MAX_SENTENCE_LENGTH + 16];
        uint16_t Nmea_len;
        uint8_t data_raw[1024];
        uint16_t data_raw_len;
    } gnss_ctrl_t;

    typedef struct
    {
        uint8_t head[8];
        uint16_t gps_week_count;
        uint32_t gps_tow_s;
        uint8_t gps_satid;         // 卫星号
        uint8_t gps_sv_accuracy;   // 用户等效距离精度 m
        uint8_t gps_sv_health;     // 卫星自主健康标识
        uint16_t gps_week;         // GPS 时间周计数，0~1023
        uint16_t gps_toe;          // GPS 卫星星历参考时间 s
        uint16_t gps_toc;          // GPS 卫星钟参考时刻  s
        double gps_af0;            // GPS 卫星钟钟差改正参数 s
        double gps_af1;            // GPS 卫星钟钟速改正参数 s/s
        double gps_af2;            // GPS 卫星钟钟漂改正参数 s/s^2
        uint8_t gps_iode;          // GPS 卫星星历数据期号
        uint16_t gps_iodc;         // GPS 卫星钟参数期卷号
        double gps_idot;           // GPS 卫星轨道倾角变化率  π/s
        double gps_crs;            // GPS 卫星轨道半径的正弦调和改正项的振幅  m
        double gps_crc;            // GPS 卫星轨道半径的余弦调和改正项的振幅 m
        double gps_cus;            // GPS 卫星纬度幅角的正弦调和改正项的振幅 rad
        double gps_cuc;            // GPS 卫星纬度幅角的余弦调和改正项的振幅 rad
        double gps_cis;            // GPS 卫星轨道倾角的正弦调和改正项的振幅 rad
        double gps_cic;            // GPS 卫星轨道倾角的余弦调和改正项的振幅 rad
        double gps_delta_n;        // GPS 卫星平均运动速率与计算值之差 π/s
        double gps_m0;             // GPS 卫星参考时间的平近点角 π
        double gps_ecc;            // GPS 卫星轨道偏心率
        double gps_a_half;         // GPS 卫星轨道长半轴的平方根 m1/2
        double gps_omega0;         // GPS 卫星按参考时间计算的升交点赤经 π
        double gps_i0;             // GPS 卫星参考时间的轨道倾角  π
        double gps_omega;          // GPS 卫星近地点幅角  π
        double gps_omegadot;       // GPS 卫星升交点赤经变化率 π/s
        double gps_tgd;            // GPS 卫星L1 和L2 信号频率的群延迟差 s
        uint8_t gps_code_on_l2;    // GPS L2测距码标志：00 = 保留；01 = P码；10 = C/A码；11 = L2C码
        uint8_t gps_l2p_data_flag; // 0=L2 P码导航电文可用；1=L2 P码导航电文不可用
        uint8_t gps_fit;           // 0=曲线拟合间隔为4小时；1=曲线拟合间隔大于4小时
        uint8_t reserved;          // 保留
        uint32_t crc24;
    } GPSEPHB_Decoded_t;

    typedef struct
    {
        uint8_t head[8];
        uint16_t gps_week_count;
        uint32_t gps_tow_s;
        uint8_t bd2_satid;
        uint8_t bd2_sv_urai;   // 用户等效距离精度指数 (URAI)
        uint8_t bd2_sv_health; // 卫星自主健康标识
        uint16_t bd2_week;     // BDS 时间周计数
        uint32_t bd2_toe;      // BDS 卫星星历参考时刻 (s)
        uint32_t bd2_toc;      // BDS 卫星钟参考时刻 (s)
        double bd2_af0;        // 卫星钟差改正参数 af0 (s)
        double bd2_af1;        // 卫星钟速改正参数 af1 (s/s)
        double bd2_af2;        // 卫星钟漂改正参数 af2 (s/s^2)
        uint8_t bd2_aode;      // 卫星星历数据期号 AODE
        uint8_t bd2_aodc;      // 卫星钟参数期卷号 AODC
        double bd2_idot;       // 轨道倾角变化率 idot (rad/s)
        double bd2_crs;        // 轨道半径正弦调和改正项 Crs (m)
        double bd2_crc;        // 轨道半径余弦调和改正项 Crc (m)
        double bd2_cus;        // 纬度幅角正弦调和改正项 Cus (rad)
        double bd2_cuc;        // 纬度幅角余弦调和改正项 Cuc (rad)
        double bd2_cis;        // 轨道倾角正弦调和改正项 Cis (rad)
        double bd2_cic;        // 轨道倾角余弦调和改正项 Cic (rad)
        double bd2_delta_n;    // 平均运动与计算值之差 Δn (rad/s)
        double bd2_m0;         // 平近点角 M0 (rad)
        double bd2_ecc;        // 轨道偏心率 e
        double bd2_a_half;     // 半长轴平方根 sqrt(A) (m^1/2)
        double bd2_omega0;     // 升交点赤经 Ω0 (rad)
        double bd2_i0;         // 轨道倾角 i0 (rad)
        double bd2_omega;      // 近地点幅角 ω (rad)
        double bd2_omegadot;   // 升交点赤经变化率 Ωdot (rad/s)
        double bd2_tgd1;       // 群延迟差 TGD1 (s)
        double bd2_tgd2;       // 群延迟差 TGD2 (s)
        uint8_t bd2_reserved;  // 保留
        uint32_t crc24;        // 嵌入的 CRC24
    } BD2EPHB_Decoded_t;

    typedef struct
    {
        uint8_t head[8];
        uint16_t gps_week_count; // GPS 周计数：用于区分数据包的时间标识（12 位原始字段）
        uint32_t gps_tow_s;      // GPS 周内秒：接收机时间戳（20 位原始字段）
        uint8_t bd3_satid;       // 卫星号：卫星 PRN/编号（6 位）
        uint8_t bd3_sattype;     // 卫星类型：GEO/MEO/IGSO 等（3 位，按截图定义）
        uint16_t bd3_week;       // BDS 周计数：北斗周（13 位原始字段，直接输出）
        uint32_t bd3_toe;        // 星历参考时刻 Toe：星历参考时间（11 位原始字段，单位在协议中需乘 300s）
        uint32_t bd3_toc;        // 钟参考时刻 Toc：卫星钟参考时间（11 位原始字段，单位在协议中需乘 300s）
        double bd3_af0;          // 卫星钟差 af0：以秒为单位的钟差修正（25 位有符号，需按 -34 指数缩放）
        double bd3_af1;          // 卫星钟速 af1：秒/秒（22 位有符号，需按 -50 指数缩放）
        double bd3_af2;          // 卫星钟漂 af2：秒/秒^2（11 位有符号，需按 -66 指数缩放）
        uint8_t bd3_iode;        // 星历数据期号 IODE（8 位）
        uint16_t bd3_iodc;       // 钟参数期卷号 IODC（16 位）
        double bd3_idot;         // 轨道倾角变化率 idot：单位为 rad/s（15 位有符号，按 -44 π/2^exp 缩放）
        double bd3_crs;          // Crs：轨道半径正弦调和改正项振幅，单位 m（24 位有符号，按 -8 缩放）
        double bd3_crc;          // Crc：轨道半径余弦调和改正项振幅，单位 m（24 位有符号，按 -8 缩放）
        double bd3_cus;          // Cus：纬度幅角正弦调和改正项振幅，单位 rad（21 位有符号，按 -30 缩放）
        double bd3_cuc;          // Cuc：纬度幅角余弦调和改正项振幅，单位 rad（21 位有符号，按 -30 缩放）
        double bd3_cis;          // Cis：轨道倾角正弦调和改正项振幅，单位 rad（16 位有符号，按 -30 缩放）
        double bd3_cic;          // Cic：轨道倾角余弦调和改正项振幅，单位 rad（16 位有符号，按 -30 缩放）
        double bd3_delta_n0;     // 平均运动修正 Δn0：单位 rad/s（17 位有符号，按 -44 π/2^exp 缩放）
        double bd3_delta_n0_dot; // Δn0 的一阶导数：单位 rad/s^2（23 位有符号，按 -57 缩放）
        double bd3_m0;           // 平近点角 M0：单位 rad（33 位无符号/有符号按截图处理，按 -32 π/2^exp 缩放）
        double bd3_ecc;          // 偏心率 e（33 位无符号，按 -34 缩放）
        double bd3_deltaA;       // 轨道长半轴修正 ΔA：单位 m（26 位无符号/按截图缩放 -9）
        double bd3_adot;         // ADOT：单位 m/s（25 位有符号，按 -21 缩放）
        double bd3_omega0;       // 升交点赤经 Ω0：单位 rad（33 位，按 -32 π/2^exp 缩放）
        double bd3_i0;           // 轨道倾角 i0：单位 rad（33 位，按 -32 π/2^exp 缩放）
        double bd3_omega;        // 近地点幅角 ω：单位 rad（33 位，按 -32 π/2^exp 缩放）
        double bd3_omegadot;     // 升交点赤经变化率 Ωdot：单位 rad/s（19 位有符号，按 -44 缩放）
        double bd3_tgdb1cp;      // TGDB1Cp：B1C 频段的群延迟差，单位 s（12 位有符号，按 -34 缩放）
        double bd3_tgdb2ap;      // TGDB2Ap：B2A 频段的群延迟差，单位 s（12 位有符号，按 -34 缩放）
        double bd3_iscb1cd;      // ISCB1Cd：卫星常数/干扰校正项，单位 s（12 位有符号，按 -34 缩放）
        uint8_t bd3_reserved;    // 保留位（2 位）
        uint32_t crc24;          // 嵌入的 CRC24（24 位）
    } BD3EPHB_Decoded_t;

    typedef struct
    {
        uint8_t head[8];
        uint16_t gps_week_count; // GPS 周计数（12 位原始字段）
        uint32_t gps_tow_s;      // GPS 周内秒（20 位原始字段）
        uint8_t glo_satid;       // GLONASS 卫星号（6 位）
        uint8_t glo_freq;        // 频率通道号（5 位）
        uint8_t glo_bn_msb;      // Bn 字的 MSB（1 位，健康标志）
        uint8_t glo_n4;          // GLO-M N4（5 位，4 年周期计数）
        uint16_t glo_nt;         // GLO-M NT（11 位，年内天数）
        uint16_t glo_tk;         // GLO tk（12 位，含小时/分钟/30s 标志）
        uint8_t glo_tb;          // GLO tb（7 位，参考时间，单位见说明）
        double glo_gamma;        // 预测的载波频率偏差 γn(tb)（11 位有符号，按 2^-40 缩放）
        double glo_tau;          // 卫星时间改正 τn(tb)（22 位有符号，按 2^-30 缩放，单位 s）
        double glo_x;            // X 坐标（Int27，按 2^-11 缩放，单位 km）
        double glo_x_dot;        // X 一阶导数（Int24，按 2^-20 缩放，单位 km/s）
        double glo_x_ddot;       // X 二阶导数（Int5，按 2^-30 缩放，单位 km/s^2）
        double glo_y;            // Y 坐标（Int27，按 2^-11 缩放，单位 km）
        double glo_y_dot;        // Y 一阶导数（Int24，按 2^-20 缩放，单位 km/s）
        double glo_y_ddot;       // Y 二阶导数（Int5，按 2^-30 缩放，单位 km/s^2）
        double glo_z;            // Z 坐标（Int27，按 2^-11 缩放，单位 km）
        double glo_z_dot;        // Z 一阶导数（Int24，按 2^-20 缩放，单位 km/s）
        double glo_z_ddot;       // Z 二阶导数（Int5，按 2^-30 缩放，单位 km/s^2）
        uint32_t crc24;          // 嵌入 CRC24（24 位）
    } GLOEPHB_Decoded_t;

    typedef struct
    {
        uint8_t head[8];
        uint16_t gps_week_count;   // GPS 周计数（12 位）
        uint32_t gps_tow_s;        // GPS 周内秒（20 位）
        uint8_t gal_satid;         // GAL 卫星号（6 位）
        uint8_t gal_sisa;          // GAL SISA Index (Uint8)
        uint8_t gal_e5b_sv_health; // E5b SV Health (2 bits)
        uint8_t gal_e5b_valid;     // E5b Data Validity (1 bit)
        uint8_t gal_e1b_health;    // E1-B SV Health (2 bits)
        uint8_t gal_e1b_valid;     // E1-B Data Validity (1 bit)
        uint16_t gal_week;         // GAL Week (12 bits)
        uint32_t gal_toe;          // GAL toe (14 bits) in s (scale *60)
        uint32_t gal_toc;          // GAL toc (14 bits) in s (scale *60)
        double gal_af0;            // Int31, scale 2^-34 s
        double gal_af1;            // Int21, scale 2^-46 s/s
        double gal_af2;            // Int6,  scale 2^-59 s/s^2
        uint16_t gal_iodnav;       // Uint10
        double gal_idot;           // Int14, scale 2^-43 π/s
        double gal_crs;            // Int16, scale 2^-5 m
        double gal_crc;            // Int16, scale 2^-5 m
        double gal_cus;            // Int16, scale 2^-29 rad
        double gal_cuc;            // Int16, scale 2^-29 rad
        double gal_cis;            // Int16, scale 2^-29 rad
        double gal_cic;            // Int16, scale 2^-29 rad
        double gal_delta_n;        // Int16, scale 2^-43 π/s
        double gal_m0;             // Int32, scale 2^-31 π
        double gal_ecc;            // Uint32, scale 2^-33
        double gal_a_half;         // Uint32, scale 2^-19 m^1/2
        double gal_omega0;         // Int32, scale 2^-31 π
        double gal_i0;             // Int32, scale 2^-31 π
        double gal_omega;          // Int32, scale 2^-31 π
        double gal_omegadot;       // Int24, scale 2^-43 π/s
        double gal_bgd_e5a_e1;     // Int10, scale 2^-32 s
        double gal_bgd_e5b_e1;     // Int10, scale 2^-32 s
        uint8_t gal_navtype;       // Uint2
        uint8_t gal_reserved;      // Uint4
        uint32_t crc24;            // Uint24
    } GALEPHB_Decoded_t;

    typedef struct
    {
        uint8_t head[8];
        uint16_t gps_week_count; // GPS 周计数（12 位）
        uint32_t gps_tow_s;      // GPS 周内秒（20 位）
        uint8_t bds_satid;       // BDS 卫星号（6 位）
        uint8_t bds_sattype;     // BDS SatType（3 位，1=GEO 2=MEO 4=IGSO）
        uint16_t bds_week;       // BDS 周计数（13 位）
        uint32_t bds_toe;        // BDS Toe（11 位，单位需乘 300s）
        uint32_t bds_toc;        // BDS Toc（11 位，单位需乘 300s）
        double bds_af0;          // Int25, scale 2^-34 s
        double bds_af1;          // Int22, scale 2^-50 s/s
        double bds_af2;          // Int11, scale 2^-66 s/s^2
        uint8_t bds_iode;        // Uint8
        uint16_t bds_iodc;       // Uint16
        double bds_idot;         // Int15, scale 2^-44 π/s
        double bds_crs;          // Int24, scale 2^-8 m
        double bds_crc;          // Int24, scale 2^-8 m
        double bds_cus;          // Int21, scale 2^-30 rad
        double bds_cuc;          // Int21, scale 2^-30 rad
        double bds_cis;          // Int16, scale 2^-30 rad
        double bds_cic;          // Int16, scale 2^-30 rad
        double bds_delta_n0;     // Int17, scale 2^-44 π/s
        double bds_delta_n0_dot; // Int23, scale 2^-57 π/s^2 (保留或备用)
        double bds_m0;           // Int33, scale 2^-32 π
        double bds_ecc;          // Uint33, scale 2^-34
        double bds_deltaA;       // Int26, scale 2^-9 m
        double bds_adot;         // Int25, scale 2^-21 m/s
        double bds_omega0;       // Int33, scale 2^-32 π
        double bds_i0;           // Int33, scale 2^-32 π
        double bds_omega;        // Int33, scale 2^-32 π
        double bds_omegadot;     // Int19, scale 2^-44 π/s
        double bds_tgdb2bI;      // Int12, scale 2^-34 s
        uint32_t bds_reserved;   // Reserved (26 bits)
        uint32_t crc24;          // Uint24
    } BD3CNAV3EPHB_Decoded_t;

    typedef struct
    {
        uint8_t head[8];
        uint16_t gps_week_count; // GPS 周计数（12 位）
        uint32_t gps_tow_s;      // GPS 周内秒（20 位）
        uint8_t bds_satid;       // BDS 卫星号（6 位）
        uint8_t bds_sattype;     // SatType（3 位）
        uint16_t bds_week;       // BDS 周计数（13 位）
        uint32_t bds_toe;        // BDS Toe（11 位，*300s）
        uint32_t bds_toc;        // BDS Toc（11 位，*300s）
        double bds_af0;          // Int25, scale 2^-34 s
        double bds_af1;          // Int22, scale 2^-50 s/s
        double bds_af2;          // Int11, scale 2^-66 s/s^2
        uint8_t bds_iode;        // Uint8
        uint16_t bds_iodc;       // Uint16
        double bds_idot;         // Int15, scale 2^-44 π/s
        double bds_crs;          // Int24, scale 2^-8 m
        double bds_crc;          // Int24, scale 2^-8 m
        double bds_cus;          // Int21, scale 2^-30 rad
        double bds_cuc;          // Int21, scale 2^-30 rad
        double bds_cis;          // Int16, scale 2^-30 rad
        double bds_cic;          // Int16, scale 2^-30 rad
        double bds_delta_n0;     // Int17, scale 2^-44 π/s
        double bds_delta_n0_dot; // Int23, scale 2^-57 (保留)
        double bds_m0;           // Int33, scale 2^-32 π
        double bds_ecc;          // Uint33, scale 2^-34
        double bds_deltaA;       // Int26, scale 2^-9 m
        double bds_adot;         // Int25, scale 2^-21 m/s
        double bds_omega0;       // Int33, scale 2^-32 π
        double bds_i0;           // Int33, scale 2^-32 π
        double bds_omega;        // Int33, scale 2^-32 π
        double bds_omegadot;     // Int19, scale 2^-44 π/s
        double bds_tgdb1cp;      // Int12, scale 2^-34 s
        double bds_tgdb2ap;      // Int12, scale 2^-34 s
        double bds_iscb1cd;      // Int12, scale 2^-34 s
        uint8_t bds_reserved2;   // Reserved (2 bits)
        uint32_t crc24;          // Uint24
    } BD3CNAV1EPHB_Decoded_t;

    typedef struct
    {
        uint8_t head[8];
        uint16_t gps_week_count; // GPS 周计数（12 位）
        uint32_t gps_tow_s;      // GPS 周内秒（20 位）
        uint8_t bds_satid;       // BDS 卫星号（6 位）
        uint8_t bds_sattype;     // SatType（3 位）
        uint16_t bds_week;       // BDS 周计数（13 位）
        uint32_t bds_toe;        // BDS Toe（11 位，*300s）
        uint32_t bds_toc;        // BDS Toc（11 位，*300s）
        double bds_af0;          // Int25, scale 2^-34 s
        double bds_af1;          // Int22, scale 2^-50 s/s
        double bds_af2;          // Int11, scale 2^-66 s/s^2
        uint8_t bds_iode;        // Uint8
        uint16_t bds_iodc;       // Uint16
        double bds_idot;         // Int15, scale 2^-44 π/s
        double bds_crs;          // Int24, scale 2^-8 m
        double bds_crc;          // Int24, scale 2^-8 m
        double bds_cus;          // Int21, scale 2^-30 rad
        double bds_cuc;          // Int21, scale 2^-30 rad
        double bds_cis;          // Int16, scale 2^-30 rad
        double bds_cic;          // Int16, scale 2^-30 rad
        double bds_delta_n0;     // Int17, scale 2^-44 π/s
        double bds_delta_n0_dot; // Int23, scale 2^-57 (保留)
        double bds_m0;           // Int33, scale 2^-32 π
        double bds_ecc;          // Uint33, scale 2^-34
        double bds_AA;           // Int26, scale 2^-9 m (A)
        double bds_adot;         // Int25, scale 2^-21 m/s
        double bds_omega0;       // Int33, scale 2^-32 π
        double bds_i0;           // Int33, scale 2^-32 π
        double bds_omega;        // Int33, scale 2^-32 π
        double bds_omegadot;     // Int19, scale 2^-44 π/s
        double bds_tgdb1cp;      // Int12, scale 2^-34 s
        double bds_tgdb2ap;      // Int12, scale 2^-34 s
        double bds_iscb2ad;      // Int12, scale 2^-34 s
        uint8_t bds_reserved;    // Uint2
        uint32_t crc24;          // Uint24
    } BD3CNAV2EPHB_Decoded_t;

/* PRANGEB: 每卫星伪距/载相/载噪比等观测/粗略信息报文解析结构 */
#define PRANGEB_MAX_SATS 64
    typedef struct
    {
        uint8_t head[8];
        uint16_t gps_week_count; // 12
        uint32_t gps_tow_s;      // 20
        uint8_t ms_count;        // 7  毫秒计数（单位见截图，1 表示 10 ms）
        uint8_t sync_flag;       // 1
        uint8_t system_id;       // 3
        uint8_t sat_count;       // 6

        /* per-satellite fields (variable length, up to sat_count) */
        uint8_t sat_id[PRANGEB_MAX_SATS];            // 6
        uint8_t signal_count[PRANGEB_MAX_SATS];      // 4
        uint8_t int_ms[PRANGEB_MAX_SATS];            // 8  概略距离的整数秒数（此处按原始字段保留）
        uint16_t frac_ms[PRANGEB_MAX_SATS];          // 10 概略距离的毫秒余数
        int16_t approx_phase_rate[PRANGEB_MAX_SATS]; // 14 Sint14 概略相位距离变化率

        /* 精确值（如果有） */
        uint8_t signal_id[PRANGEB_MAX_SATS];          // 6
        uint8_t phase_lock_flag[PRANGEB_MAX_SATS];    // 4
        int16_t precise_pr[PRANGEB_MAX_SATS];         // 15 Sint15, scale 2^-24 ms
        int32_t precise_phase[PRANGEB_MAX_SATS];      // 22 Sint22, scale 2^-29 ms
        int16_t precise_phase_rate[PRANGEB_MAX_SATS]; // 15 Sint15, scale 10^-4 m/s
        uint16_t cn0[PRANGEB_MAX_SATS];               // 13, scale 10^-2 dB
        uint8_t half_cycle[PRANGEB_MAX_SATS];         // 1
        uint8_t reserved[PRANGEB_MAX_SATS];           // 4 bits

        uint32_t crc24; // 嵌入的 CRC24（位于 payload 最末 3 字节）
    } PRANGEB_Decoded_t;

    /* PRANGE2B: 另一个伪距报文变体，结构与 PRANGEB 类似，保留扩展字段占位 */
    typedef struct
    {
        uint8_t head[8];
        uint16_t gps_week_count; // 12
        uint32_t gps_tow_s;      // 20
        uint8_t ms_count;        // 7
        uint8_t sync_flag;       // 1
        uint8_t system_id;       // 3
        uint8_t sat_count;       // 6

        /* per-satellite base fields */
        uint8_t sat_id[PRANGEB_MAX_SATS];
        uint8_t signal_count[PRANGEB_MAX_SATS];
        uint8_t int_ms[PRANGEB_MAX_SATS];
        uint16_t frac_ms[PRANGEB_MAX_SATS];
        int16_t approx_phase_rate[PRANGEB_MAX_SATS];

        /* PRANGE2B 特有扩展（按保守方式保存） */
        uint8_t ext_flag[PRANGEB_MAX_SATS];
        uint8_t signal_id[PRANGEB_MAX_SATS];
        uint8_t phase_lock_flag[PRANGEB_MAX_SATS];
        int16_t precise_pr[PRANGEB_MAX_SATS];
        int32_t precise_phase[PRANGEB_MAX_SATS];
        int16_t precise_phase_rate[PRANGEB_MAX_SATS];
        uint16_t cn0[PRANGEB_MAX_SATS];
        uint8_t half_cycle[PRANGEB_MAX_SATS];
        uint8_t reserved[PRANGEB_MAX_SATS];

        uint32_t crc24;
    } PRANGE2B_Decoded_t;

    void decode_gpsephb(const uint8_t *payload, size_t payload_len, GPSEPHB_Decoded_t *out);
    void decode_bd2ephb(const uint8_t *payload, size_t payload_len, BD2EPHB_Decoded_t *out);
    void decode_bd3ephb(const uint8_t *payload, size_t payload_len, BD3EPHB_Decoded_t *out);
    void decode_gloephb(const uint8_t *payload, size_t payload_len, GLOEPHB_Decoded_t *out);
    void decode_galephb(const uint8_t *payload, size_t payload_len, GALEPHB_Decoded_t *out);
    void decode_bd3cnav1ephb(const uint8_t *payload, size_t payload_len, BD3CNAV1EPHB_Decoded_t *out);
    void decode_bd3cnav2ephb(const uint8_t *payload, size_t payload_len, BD3CNAV2EPHB_Decoded_t *out);
    void decode_bd3cnav3ephb(const uint8_t *payload, size_t payload_len, BD3CNAV3EPHB_Decoded_t *out);
    void decode_prangeb(const uint8_t *payload, size_t payload_len, PRANGEB_Decoded_t *out);
    void decode_prange2b(const uint8_t *payload, size_t payload_len, PRANGE2B_Decoded_t *out);

    extern gnss_ctrl_t gnss_ctrl;

    void handle_gnss_nmea(const char *sentence);
    int handle_gnss_raw(const uint8_t *data, size_t len);
    void gnss_cfg_close_all(int fd);
    void gnss_cfg_eable_onchange(int fd, char *type);
    void gnss_cfg(int fd, char *type, uint8_t enable, uint8_t per_second);
    int8_t gnss_bdd_enable();
    int8_t gnss_bdd_disable();

#ifdef __cplusplus
}
#endif
