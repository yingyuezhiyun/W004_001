#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "third_party/minmea.h"
    typedef enum
    {
        GNSS_DATA_AUTO,
        GNSS_DATA_NMEA,
        GNSS_DATA_RAW,
    } gnss_DataType_t;

    typedef struct
    {
        int fd;
        gnss_DataType_t data_type;
        char data_nmea[MINMEA_MAX_SENTENCE_LENGTH + 16];
        uint16_t data_nmea_len;
        uint8_t data_raw[1024];
        uint16_t data_raw_len;
    } gnss_ctrl_t;

    typedef struct
    {
        uint8_t head[8];
        uint16_t gps_week_count;   // 占位12；输出数据时刻，GPS 时间，周计数为实际值；-
        uint32_t gps_tow_s;        // 占位20；GPS 周内秒；-
        uint8_t gps_satid;         // 占位6；按卫星号从小到大顺序输出；-
        uint8_t gps_sv_accuracy;   // 占位4；用户等效距离精度；m
        uint8_t gps_sv_health;     // 占位6；卫星自主健康标识；-
        uint16_t gps_week;         // 占位10；GPS 时间周计数，0~1023；-
        uint16_t gps_toe;          // 占位16；GPS 卫星星历参考时间；s
        uint16_t gps_toc;          // 占位16；GPS 卫星钟参考时刻；s
        double gps_af0;            // 占位22；GPS 卫星钟钟差改正参数；s
        double gps_af1;            // 占位16；GPS 卫星钟钟速改正参数；s/s
        double gps_af2;            // 占位8；GPS 卫星钟钟漂改正参数；s/s^2
        uint8_t gps_iode;          // 占位8；GPS 卫星星历数据期号；-
        uint16_t gps_iodc;         // 占位10；GPS 卫星钟参数期卷号；-
        double gps_idot;           // 占位14；GPS 卫星轨道倾角变化率；π/s
        double gps_crs;            // 占位16；GPS 卫星轨道半径的正弦调和改正项的振幅；m
        double gps_crc;            // 占位16；GPS 卫星轨道半径的余弦调和改正项的振幅；m
        double gps_cus;            // 占位16；GPS 卫星纬度幅角的正弦调和改正项的振幅；rad
        double gps_cuc;            // 占位16；GPS 卫星纬度幅角的余弦调和改正项的振幅；rad
        double gps_cis;            // 占位16；GPS 卫星轨道倾角的正弦调和改正项的振幅；rad
        double gps_cic;            // 占位16；GPS 卫星轨道倾角的余弦调和改正项的振幅；rad
        double gps_delta_n;        // 占位14；GPS 卫星平均运动速率与计算值之差；π/s
        double gps_m0;             // 占位22；GPS 卫星参考时间的平近点角；π
        double gps_ecc;            // 占位33；GPS 卫星轨道偏心率；-
        double gps_a_half;         // 占位32；GPS 卫星轨道长半轴的平方根；m1/2
        double gps_omega0;         // 占位32；GPS 卫星按参考时间计算的升交点赤经；π
        double gps_i0;             // 占位32；GPS 卫星参考时间的轨道倾角；π
        double gps_omega;          // 占位32；GPS 卫星近地点幅角；π
        double gps_omegadot;       // 占位24；GPS 卫星升交点赤经变化率；π/s
        double gps_tgd;            // 占位8；GPS 卫星 L1 和 L2 信号频率的群延迟差；s
        uint8_t gps_code_on_l2;    // 占位2；GPS L2 测距码标志：00 = 保留；01 = P 码；10 = C/A 码；11 = L2C 码；-
        uint8_t gps_l2p_data_flag; // 占位1；0=L2 P 码导航电文可用；1=L2 P 码导航电文不可用；-
        uint8_t gps_fit;           // 占位1；0=曲线拟合间隔为 4 小时；1=曲线拟合间隔大于 4 小时；-
        uint8_t reserved;          // 占位4；保留；-
        uint32_t crc24;
    } GPSEPHB_Decoded_t;

    typedef struct
    {
        uint8_t head[8];
        uint16_t gps_week_count; // GPS周计数
        uint32_t gps_tow_s;      // GPS周内秒
        uint8_t bd2_satid;       // 卫星号
        uint8_t bd2_sv_urai;     // 用户等效距离精度指数 (URAI)
        uint8_t bd2_sv_health;   // 卫星自主健康标识
        uint16_t bd2_week;       // BDS 时间周计数
        uint32_t bd2_toe;        // BDS 卫星星历参考时刻 (s)
        uint32_t bd2_toc;        // BDS 卫星钟参考时刻 (s)
        double bd2_af0;          // BDS 卫星钟钟差改正参数  (s)
        double bd2_af1;          // BDS 卫星钟速改正参数 af1 (s/s)
        double bd2_af2;          // BDS 卫星钟漂改正参数 af2 (s/s^2)
        uint8_t bd2_aode;        // BDS 卫星星历数据龄期 AODE (s)
        uint8_t bd2_aodc;        // BDS 卫星钟时钟数据龄期 AODC (s)
        double bd2_idot;         // BDS 卫星轨道倾角变化率  idot (π/s)
        double bd2_crs;          // BDS 卫星轨道半径正弦调和改正项的振幅  Crs (m)
        double bd2_crc;          // BDS 轨道半径余弦调和改正项的振幅 Crc (m)
        double bd2_cus;          // BDS 纬度幅角正弦调和改正项的振幅 Cus (rad)
        double bd2_cuc;          // BDS 纬度幅角余弦调和改正项的振幅 Cuc (rad)
        double bd2_cis;          // BDS 轨道倾角正弦调和改正项的振幅 Cis (rad)
        double bd2_cic;          // BDS 轨道倾角余弦调和改正项的振幅 Cic (rad)
        double bd2_delta_n;      // BDS 卫星平均运动速率与计算值之差 Δn (π/s)
        double bd2_m0;           // BDS 卫星参考时间的平近点角 M0 (π)
        double bd2_ecc;          // BDS 卫星轨道偏心率  e
        double bd2_a_half;       // BDS 卫星轨道长半轴的平方根 sqrt(A) (m^1/2)
        double bd2_omega0;       // BDS 卫星按参考时间计算的升交点赤经  Ω0 (π)
        double bd2_i0;           // BDS 卫星参考时间的轨道倾角 i0 (π)
        double bd2_omega;        // BDS 卫星近地点幅角  ω (π)
        double bd2_omegadot;     // BDS 卫星升交点赤经变化率 Ωdot (π/s)
        double bd2_tgd1;         // BDS 卫星B1I 星上设备时延差  TGD1 (ns)
        double bd2_tgd2;         // BDS 卫星B2I 星上设备时延差 TGD2 (ns)
        uint8_t bd2_reserved;    // 保留
        uint32_t crc24;          // 嵌入的 CRC24
    } BD2EPHB_Decoded_t;

    typedef struct
    {
        uint8_t head[8];
        uint16_t gps_week_count; // 占位12；输出数据时刻，GPS 时间，周计数为实际值；-
        uint32_t gps_tow_s;      // 占位20；GPS 周内秒；-
        uint8_t bd3_satid;       // 占位6；按卫星号从小到大顺序输出；-
        uint8_t bd3_sattype;     // 占位3；1-GEO 2-MEO 4-IGSO；-
        uint16_t bd3_week;       // 占位13；BDS 时间周计数；-
        uint32_t bd3_toe;        // 占位11；BDS 卫星星历数据参考时刻；s
        uint32_t bd3_toc;        // 占位11；BDS 卫星钟数据参考时刻；s
        double bd3_af0;          // 占位25；BDS 卫星钟钟差改正参数；s
        double bd3_af1;          // 占位22；BDS 卫星钟钟速改正参数；s/s
        double bd3_af2;          // 占位11；BDS 卫星钟钟漂改正参数；s/s2
        uint8_t bd3_iode;        // 占位8；BDS 卫星星历数据龄期,注意转 RINEX 时转为 AODE；-
        uint16_t bd3_iodc;       // 占位16；BDS 卫星时钟数据龄期；-
        double bd3_idot;         // 占位15；BDS 卫星轨道倾角变化率；π/s
        double bd3_crs;          // 占位24；BDS 卫星轨道半径正弦调和改正项的振幅；m
        double bd3_crc;          // 占位24；BDS 卫星轨道半径的余弦调和改正项的振幅；m
        double bd3_cus;          // 占位21；BDS 卫星纬度幅角的正弦调和改正项的振幅；rad
        double bd3_cuc;          // 占位21；BDS 卫星纬度幅角的余弦调和改正项的振幅；rad
        double bd3_cis;          // 占位16；BDS 卫星轨道倾角的正弦调和改正项的振幅；rad
        double bd3_cic;          // 占位16；BDS 卫星轨道倾角的余弦调和改正项的振幅；rad
        double bd3_delta_n0;     // 占位17；BDS 卫星平均运动速率与计算值之差；π/s
        double bd3_delta_n0_dot; // 占位23；RINEX 中没有，需放在保留字段处；π/s2
        double bd3_m0;           // 占位33；BDS 卫星参考时间的平近点角；π
        double bd3_ecc;          // 占位33；BDS 卫星轨道偏心率；-
        double bd3_deltaA;       // 占位26；与 SatType 共同计算得到(A)1/2；m
        double bd3_adot;         // 占位25；BDS ADOT；m/s
        double bd3_omega0;       // 占位33；BDS 卫星按参考时间计算的升交点赤经；π
        double bd3_i0;           // 占位33；BDS 卫星参考时间的轨道倾角；π
        double bd3_omega;        // 占位33；BDS 卫星近地点幅角；π
        double bd3_omegadot;     // 占位19；BDS 卫星升交点赤经变化率；π/s
        double bd3_tgdb1cp;      // 占位12；B1C 导频分量试时延差；s
        double bd3_tgdb2ap;      // 占位12；B2A 导频分量试时延差；s
        double bd3_iscb1cd;      // 占位12；B1C 数据分量相对于 B1C 导频分量的时延修正项；s
        uint8_t bd3_reserved;    // 占位2；保留；-
        uint32_t crc24;          // 占位24；校验范围从帧头至有效数据；-
    } BD3EPHB_Decoded_t;

    typedef struct
    {
        uint8_t head[8];
        uint16_t gps_week_count; // 占位12；输出数据时刻，GPS 时间，周计数为实际值；-
        uint32_t gps_tow_s;      // 占位20；GPS 周内秒；-
        uint8_t glo_satid;       // 占位6；按卫星号从小到大顺序输出；-
        uint8_t glo_freq;        // 占位5；GLONASS 卫星的频率通道号：0=-07；1=-06；..；19=+12；20=+13；-
        uint8_t glo_bn_msb;      // 占位1；星历健康状况标志；-
        uint8_t glo_n4;          // 占位5；从 1996 年开始的，以 4 年为周期的周期数；year
        uint16_t glo_nt;         // 占位11；以四年为间隔，从闰年的一月一日开始的日历天数；day
        uint16_t glo_tk;         // 占位12；当天 GLONASS 子帧的起点为零点的时间。（最高有 5 位）MSB5 为小时数（整数），之后的 6 位为分钟数（整数），最低有效位（LSB）为 30 秒的采样间隔数；-
        uint8_t glo_tb;          // 占位7；GLONASS 导航数据的参考时间；min
        double glo_gamma;        // 占位11；预计的 GLONASS 卫星载波频率导数（相对于名义值）；-
        double glo_tau;          // 占位22；相对 GLONASS 系统时间的卫星时间改正值；s
        double glo_x;            // 占位27；用于组成 PZ-90 坐标系下 GLONASS 卫星速度矢量的 X 分量；km
        double glo_x_dot;        // 占位24；用于组成 PZ-90 坐标系下 GLONASS 卫星速度矢量的 X 分量；km/s
        double glo_x_ddot;       // 占位5；用于组成 PZ-90 坐标系下 GLONASS 卫星速度矢量的 X 分量；km/s2
        double glo_y;            // 占位27；用于组成 PZ-90 坐标系下 GLONASS 卫星速度矢量的 Y 分量；km
        double glo_y_dot;        // 占位24；用于组成 PZ-90 坐标系下 GLONASS 卫星速度矢量的 Y 分量；km/s
        double glo_y_ddot;       // 占位5；用于组成 PZ-90 坐标系下 GLONASS 卫星速度矢量的 Y 分量；km/s2
        double glo_z;            // 占位27；用于组成 PZ-90 坐标系下 GLONASS 卫星速度矢量的 Z 分量；km
        double glo_z_dot;        // 占位24；用于组成 PZ-90 坐标系下 GLONASS 卫星速度矢量的 Z 分量；km/s
        double glo_z_ddot;       // 占位5；用于组成 PZ-90 坐标系下 GLONASS 卫星速度矢量的 Z 分量；km/s2
        uint32_t crc24;          // 占位24；校验范围从帧头至有效数据；-
    } GLOEPHB_Decoded_t;

    typedef struct
    {
        uint8_t head[8];
        uint16_t gps_week_count;   // 占位12；输出数据时刻，GPS 时间，周计数为实际值；-
        uint32_t gps_tow_s;        // 占位20；GPS 周内秒；-
        uint8_t gal_satid;         // 占位6；按卫星号从小到大顺序输出；-
        uint8_t gal_sisa;          // 占位8；空间信号精度：0~49 = 0cm~49cm；-
        uint8_t gal_e5b_sv_health; // 占位2；0 = 信号正常；1 = 信号停止服务；2 = 信号将停止服务；3 = 信号处于测试状态；-
        uint8_t gal_e5b_valid;     // 占位1；E5b 导航数据有效状态标志；-
        uint8_t gal_e1b_health;    // 占位2；0 = 信号正常；1 = 信号停止服务；2 = 信号将停止服务；3 = 信号处于测试状态；-
        uint8_t gal_e1b_valid;     // 占位1；E1-B 导航数据有效状态标志；-
        uint16_t gal_week;         // 占位12；Galileo 系统时间周计数；week
        uint32_t gal_toe;          // 占位14；GAL 卫星星历参考时间；s
        uint32_t gal_toc;          // 占位14；GAL 卫星钟参考时刻；s
        double gal_af0;            // 占位31；GAL 卫星钟钟差改正参数；s
        double gal_af1;            // 占位21；GAL 卫星钟钟速改正参数；s/s
        double gal_af2;            // 占位6；GAL 卫星钟钟漂改正参数；s/s2
        uint16_t gal_iodnav;       // 占位10；GAL 导航数据的期卷号；-
        double gal_idot;           // 占位14；GAL 卫星轨道倾角变化率；π/s
        double gal_crs;            // 占位16；GAL 卫星轨道半径的正弦调和改正项的振幅；m
        double gal_crc;            // 占位16；GAL 卫星轨道半径的余弦调和改正项的振幅；m
        double gal_cus;            // 占位16；GAL 卫星纬度幅角的正弦调和改正项的振幅；rad
        double gal_cuc;            // 占位16；GAL 卫星纬度幅角的余弦调和改正项的振幅；rad
        double gal_cis;            // 占位16；GAL 卫星轨道倾角的正弦调和改正项的振幅；rad
        double gal_cic;            // 占位16；GAL 卫星轨道倾角的余弦调和改正项的振幅；rad
        double gal_delta_n;        // 占位16；GAL 卫星平均运动速率与计算值之差；π/s
        double gal_m0;             // 占位32；GAL 卫星参考时间的平近点角；π
        double gal_ecc;            // 占位32；GAL 卫星轨道偏心率；-
        double gal_a_half;         // 占位32；GAL 卫星轨道长半轴的平方根；m1/2
        double gal_omega0;         // 占位32；GAL 卫星按参考时间计算的升交点赤经；π
        double gal_i0;             // 占位32；GAL 卫星参考时间的轨道倾角；π
        double gal_omega;          // 占位32；GAL 卫星近地点幅角；π
        double gal_omegadot;       // 占位24；GAL 卫星升交点赤经变化率；π/s
        double gal_bgd_e5a_e1;     // 占位10；GAL E1/E5a 的群延迟；s
        double gal_bgd_e5b_e1;     // 占位10；GAL E1/E5b 的群延迟；s
        uint8_t gal_navtype;       // 占位2；星历类型标志：1-E1 星历；2-E5b 星历；3-E1+E5b 星历；-
        uint8_t gal_reserved;      // 占位4；保留位；-
        uint32_t crc24;            // 占位24；校验范围从帧头至有效数据；-
    } GALEPHB_Decoded_t;

    /* BDXWEPHB: XWS/BDXW 星历（根据截图字段顺序与比例因子） */
    typedef struct
    {
        uint8_t head[8];
        uint16_t gps_week_count; // 占位12；输出数据时刻，GPS 时间，周计数为实际值；-
        uint32_t gps_tow_s;      // 占位20；GPS 周内秒；s
        uint8_t xws_satid;       // 占位8；按卫星号从小到大顺序输出（1~168）；-
        uint8_t xws_sattype;     // 占位2；卫星类型；-
        uint16_t xws_week;       // 占位13；XWS 时间周计数；-
        uint32_t xws_toe;        // 占位16；参考时刻 Toe，单位 12 s；s
        uint32_t xws_toc;        // 占位17；参考时刻 Toc，单位 6 s；s
        double xws_af0;          // 占位27；卫星钟偏差系数；比例因子 2^-36 s
        double xws_af1;          // 占位22；卫星钟漂移系数；比例因子 2^-50 s/s
        uint8_t xws_iodc;        // 占位4；星历/钟参数版本号；-
        uint8_t xws_iode;        // 占位3；星历版本号；-
        double xws_crs;          // 占位26；轨道半径正弦调和改正项振幅；比例因子 2^-11 m
        double xws_crc;          // 占位26；轨道半径余弦调和改正项振幅；比例因子 2^-11 m
        double xws_cus;          // 占位25；纬度幅角正弦调和改正项振幅；比例因子 2^-34 π
        double xws_cuc;          // 占位25；纬度幅角余弦调和改正项振幅；比例因子 2^-34 π
        double xws_cis;          // 占位21；轨道倾角正弦调和改正项振幅；比例因子 2^-34 π
        double xws_cic;          // 占位21；轨道倾角余弦调和改正项振幅；比例因子 2^-34 π
        double xws_deltacic;     // 占位21；纬度幅角的三次余弦改正项振幅；比例因子 2^-34 π
        double xws_deltacis;     // 占位21；纬度幅角的三次正弦改正项振幅；比例因子 2^-34 π
        double xws_deltacrs;     // 占位26；轨道半径三次余弦改正项振幅；比例因子 2^-11 π
        double xws_deltacrc;     // 占位26；轨道半径三次正弦改正项振幅；比例因子 2^-11 π
        double xws_delta_n0;     // 占位27；卫星平均运动速率与计算值之差；比例因子 2^-45 π/s
        double xws_delta_n0_dot; // 占位33；卫星平均运动速率变化率；比例因子 2^-54 π/s^2
        double xws_m0;           // 占位36；参考时刻的平近点角；比例因子 2^-35 π
        double xws_ecc;          // 占位30；轨道偏心率；比例因子 2^-35
        double xws_deltaA0;      // 占位27；相对参考值 Aref 的偏差；比例因子 2^-10 m
        double xws_i0_dot;       // 占位22；轨道倾角变化率；比例因子 2^-45 π/s
        double xws_a_dot;        // 占位22；长半轴变化率；比例因子 2^-19 m/s
        double xws_delta_i0;     // 占位31；相对参考值 iref 偏差；比例因子 2^-35 π
        double xws_omega0;       // 占位36；升交点赤经；比例因子 2^-35 π
        double xws_omega_dot;    // 占位26；升交点赤经变化率；比例因子 2^-45 π/s
        double xws_omega;        // 占位36；近地点幅角；比例因子 2^-35 π
        uint64_t xws_reserved;   // 占位36；保留位
        uint32_t crc24;          // 占位24；CRC24 校验
    } BDXWEPHB_Decoded_t;

    typedef struct
    {
        uint8_t head[8];
        uint16_t gps_week_count; // 占位12；输出数据时刻，GPS 时间，周计数为实际值；-
        uint32_t gps_tow_s;      // 占位20；GPS 周内秒；-
        uint8_t bds_satid;       // 占位6；按卫星号从小到大顺序输出；-
        uint8_t bds_sattype;     // 占位3；1-GEO 2-MEO 4-IGSO；-
        uint16_t bds_week;       // 占位13；BDS 时间周计数；-
        uint32_t bds_toe;        // 占位11；BDS 卫星星历数据参考时刻；s
        uint32_t bds_toc;        // 占位11；BDS 卫星钟数据参考时刻；s
        double bds_af0;          // 占位25；BDS 卫星钟钟差改正参数；s
        double bds_af1;          // 占位22；BDS 卫星钟钟速改正参数；s/s
        double bds_af2;          // 占位11；BDS 卫星钟钟漂改正参数；s/s2
        uint8_t bds_iode;        // 占位8；BDS 卫星星历数据龄期,注意转 RINEX 时转为 AODE；-
        uint16_t bds_iodc;       // 占位16；BDS 卫星时钟数据龄期；-
        double bds_idot;         // 占位15；BDS 卫星轨道倾角变化率；π/s
        double bds_crs;          // 占位24；BDS 卫星轨道半径正弦调和改正项的振幅；m
        double bds_crc;          // 占位24；BDS 卫星轨道半径的余弦调和改正项的振幅；m
        double bds_cus;          // 占位21；BDS 卫星纬度幅角的正弦调和改正项的振幅；rad
        double bds_cuc;          // 占位21；BDS 卫星纬度幅角的余弦调和改正项的振幅；rad
        double bds_cis;          // 占位16；BDS 卫星轨道倾角的正弦调和改正项的振幅；rad
        double bds_cic;          // 占位16；BDS 卫星轨道倾角的余弦调和改正项的振幅；rad
        double bds_delta_n0;     // 占位17；BDS 卫星平均运动速率与计算值之差；π/s
        double bds_delta_n0_dot; // 占位23；RINEX 中没有，需放在保留字段处；π/s2
        double bds_m0;           // 占位33；BDS 卫星参考时间的平近点角；π
        double bds_ecc;          // 占位33；BDS 卫星轨道偏心率；-
        double bds_deltaA;       // 占位26；与 SatType 共同计算得到(A)1/2；m
        double bds_adot;         // 占位25；BDS ADOT；m/s
        double bds_omega0;       // 占位33；BDS 卫星按参考时间计算的升交点赤经；π
        double bds_i0;           // 占位33；BDS 卫星参考时间的轨道倾角；π
        double bds_omega;        // 占位33；BDS 卫星近地点幅角；π
        double bds_omegadot;     // 占位19；BDS 卫星升交点赤经变化率；π/s
        double bds_tgdb2bI;      // 占位12；B2b I 支路时延差；s
        uint32_t bds_reserved;   // 占位26；保留；-
        uint32_t crc24;          // 占位24；校验范围从帧头至有效数据；-
    } BD3CNAV3EPHB_Decoded_t;

    typedef struct
    {
        uint8_t head[8];
        uint16_t gps_week_count; // 占位12；输出数据时刻，GPS 时间，周计数为实际值；-
        uint32_t gps_tow_s;      // 占位20；GPS 周内秒；-
        uint8_t bds_satid;       // 占位6；按卫星号从小到大顺序输出；-
        uint8_t bds_sattype;     // 占位3；1-GEO 2-MEO 4-IGSO；-
        uint16_t bds_week;       // 占位13；BDS 时间周计数；-
        uint32_t bds_toe;        // 占位11；BDS 卫星星历数据参考时刻；s
        uint32_t bds_toc;        // 占位11；BDS 卫星钟数据参考时刻；s
        double bds_af0;          // 占位25；BDS 卫星钟钟差改正参数；s
        double bds_af1;          // 占位22；BDS 卫星钟钟速改正参数；s/s
        double bds_af2;          // 占位11；BDS 卫星钟钟漂改正参数；s/s2
        uint8_t bds_iode;        // 占位8；BDS 卫星星历数据龄期,注意转 RINEX 时转为 AODE；-
        uint16_t bds_iodc;       // 占位16；BDS 卫星时钟数据龄期；-
        double bds_idot;         // 占位15；BDS 卫星轨道倾角变化率；π/s
        double bds_crs;          // 占位24；BDS 卫星轨道半径正弦调和改正项的振幅；m
        double bds_crc;          // 占位24；BDS 卫星轨道半径的余弦调和改正项的振幅；m
        double bds_cus;          // 占位21；BDS 卫星纬度幅角的正弦调和改正项的振幅；rad
        double bds_cuc;          // 占位21；BDS 卫星纬度幅角的余弦调和改正项的振幅；rad
        double bds_cis;          // 占位16；BDS 卫星轨道倾角的正弦调和改正项的振幅；rad
        double bds_cic;          // 占位16；BDS 卫星轨道倾角的余弦调和改正项的振幅；rad
        double bds_delta_n0;     // 占位17；BDS 卫星平均运动速率与计算值之差；π/s
        double bds_delta_n0_dot; // 占位23；RINEX 中没有，需放在保留字段处；π/s2
        double bds_m0;           // 占位33；BDS 卫星参考时间的平近点角；π
        double bds_ecc;          // 占位33；BDS 卫星轨道偏心率；-
        double bds_AA;           // 占位26；与 SatType 共同计算得到(A)1/2；m
        double bds_adot;         // 占位25；BDS ADOT；m/s
        double bds_omega0;       // 占位33；BDS 卫星按参考时间计算的升交点赤经；π
        double bds_i0;           // 占位33；BDS 卫星参考时间的轨道倾角；π
        double bds_omega;        // 占位33；BDS 卫星近地点幅角；π
        double bds_omegadot;     // 占位19；BDS 卫星升交点赤经变化率；π/s
        double bds_tgdb1cp;      // 占位12；B1C 导频分量试时延差；s
        double bds_tgdb2ap;      // 占位12；B2A 导频分量试时延差；s
        double bds_iscb2ad;      // 占位12；B2A 数据分量相对于 B2A 导频分量的时延修正项；s
        uint8_t bds_reserved;    // 占位2；保留；-
        uint32_t crc24;          // 占位24；校验范围从帧头至有效数据；-
    } BD3CNAV2EPHB_Decoded_t;

    typedef struct
    {
        uint8_t head[12];
        uint8_t time_system;  // 占位3；0-无效系统，1-BDS，2-GPS，3-GAL，4-GLO，5~7-保留；-
        uint16_t week_num;    // 占位13；ID4=1 时为 BDS 周计数，ID4=2 时为 GPS 周计数，无效时为 0x1FFF；-
        uint32_t sec_in_week; // 占位32；ID4=1 时为 BDS 周内秒，ID4=2 时为 GPS 周内秒，无效时为 0xFFFFFFFF；s
        int8_t leap_second;   // 占位8；对应时间系统与 UTC 时间差，高位 1 表示负，无效时为 0x80；s
        double accel_x;       // 占位17；加速度计 X 轴，高位 1 表示负，无效时取 0x10000；m/s^2
        double accel_y;       // 占位17；加速度计 Y 轴，高位 1 表示负，无效时取 0x10000；m/s^2
        double accel_z;       // 占位17；加速度计 Z 轴，高位 1 表示负，无效时取 0x10000；m/s^2
        double gyro_x;        // 占位20；陀螺仪 X 轴，高位 1 表示负，无效时取 0x80000；°/s
        double gyro_y;        // 占位20；陀螺仪 Y 轴，高位 1 表示负，无效时取 0x80000；°/s
        double gyro_z;        // 占位20；陀螺仪 Z 轴，高位 1 表示负，无效时取 0x80000；°/s
        double mag_x;         // 占位19；磁力计 X 轴，高位 1 表示负，无效时取 0x40000；gauss
        double mag_y;         // 占位19；磁力计 Y 轴，高位 1 表示负，无效时取 0x40000；gauss
        double mag_z;         // 占位19；磁力计 Z 轴，高位 1 表示负，无效时取 0x40000；gauss
        uint32_t reserved0;   // 占位32；保留位，无效时为 0xFFFFFFFF；-
        uint32_t reserved1;   // 占位32；保留位，无效时为 0xFFFFFFFF；-
        uint32_t crc24;       // 占位24；CRC24 校验，校验范围从帧头至有效数据；-
    } IMUDATAB_Decoded_t;

    typedef struct
    {
        uint8_t head[12];
        uint8_t time_system;    // 占位3；时间系统：0-无效系统，1-BDS，2-GPS，3-GAL，4-GLO，5~7-保留；-
        uint16_t week_num;      // 占位13；周计数：ID4=1 时为 BDS 周计数，ID4=2 时为 GPS 周计数，无效时为 0x1FFF；-
        double sec_in_week;     // 占位32；周内秒：ID4=1 时为 BDS 周内秒，ID4=2 时为 GPS 周内秒，无效时为 0xFFFFFFFF；s
        int8_t leap_second;     // 占位8；UTC 差：对应时间系统与 UTC 时间差，高位 1 表示负，无效时为 0x80；s
        uint8_t sys_status;     // 占位4；系统状态：0-未定位 1-纯卫导 2-组合导航 3-纯惯导 4~15-保留；-
        uint8_t pos_type;       // 占位6；卫导定位状态：0-无效解 1-单点解 2-伪距差分 3-PPS 4-固定解 5-浮点解 6-航位推算 7-用户输入 8-PPP 9~63-保留；-
        uint8_t azi_type;       // 占位6；卫导定向状态：0-无效解 1-单点解 2-伪距差分 3-PPS 4-固定解 5-浮点解 6-航位推算 7-用户输入 8-PPP 9~63-保留；-
        double latitude;        // 占位36；纬度：单位度，北纬为正，南纬为负；度
        double longitude;       // 占位36；经度：单位度，东经为正，西经为负；度
        double altitude;        // 占位30；大地高：单位 m；m
        double east_velocity;   // 占位16；东向速度：ENU 下东向速度，单位 m/s；m/s
        double north_velocity;  // 占位16；北向速度：ENU 下北向速度，单位 m/s；m/s
        double up_velocity;     // 占位16；天向速度：ENU 下对天速度，单位 m/s；m/s
        double pitch;           // 占位16；俯仰角：范围[-90,+90]，仰角为正，俯角为负；度
        double roll;            // 占位16；横滚角：范围[-180,+180]，右倾为正，左倾为负；度
        double azimuth;         // 占位16；航向角：范围[0,360]；度
        double lat_sigma;       // 占位18；纬度标准差：单位 m；m
        double lon_sigma;       // 占位18；经度标准差：单位 m；m
        double altitude_sigma;  // 占位18；高程标准差：单位 m；m
        double east_vel_sigma;  // 占位12；东向速度标准差：单位 m/s；m/s
        double north_vel_sigma; // 占位12；北向速度标准差：单位 m/s；m/s
        double up_vel_sigma;    // 占位12；天向速度标准差：单位 m/s；m/s
        double pitch_sigma;     // 占位12；俯仰角标准差：单位度；度
        double roll_sigma;      // 占位12；横滚角标准差：单位度；度
        double azimuth_sigma;   // 占位12；航向角标准差：单位度；度
        uint8_t gnss_sat_m;     // 占位8；主天线卫星数：单位 颗，无效时为 0xFF；-
        uint8_t gnss_sat_s;     // 占位8；从天线卫星数：单位 颗，无效时为 0xFF；-
        double diff_age;        // 占位16；差分龄期：单位 s，无效时为 0xFFFF；s
        uint8_t odo_flag;       // 占位1；里程计标志：1-里程计速度有效，0-里程计速度无效；-
        uint8_t gear;           // 占位3；档位：1-N，1-D，2-R，3-P，4~6-备用，无效时取 7；-
        double fl_wheel_speed;  // 占位16；前左轮速：单位 m/s；m/s
        double fr_wheel_speed;  // 占位16；前右轮速：单位 m/s；m/s
        double rl_wheel_speed;  // 占位16；后左轮速：单位 m/s；m/s
        double rr_wheel_speed;  // 占位16；后右轮速：单位 m/s；m/s
        uint32_t reserved0;     // 占位32；保留位，无效时为 0xFFFFFFFF；-
        uint32_t reserved1;     // 占位32；保留位，无效时为 0xFFFFFFFF；-
        uint32_t crc24;         // 占位24；CRC24 校验，校验范围从帧头至有效数据；-
    } POSDATAB_Decoded_t;

/* PRANGEB: 每卫星伪距/载相/载噪比等观测/粗略信息报文解析结构 */
#define PRANGEB_MAX_SATS 64

    typedef struct
    {
        uint8_t head[8];
        uint16_t gps_week_count; // 占位12；当前 GPS 周计数；week
        uint32_t gps_tow_s;      // 占位20；当前 GPS 周内秒；s
        uint8_t ms_count;        // 占位7；1 表示 10 ms；ms
        uint8_t sync_flag;       // 占位1；1-非最后一条 0-当前时刻最后一条；-
        uint8_t system_id;       // 占位3；0-BDS 1-GPS 2-GLO 3-GAL；-
        uint8_t sat_count;       // 占位6；该条语句包含卫星总数；-
        // 保留7bit
        struct
        {
            uint8_t sat_id;            // 占位6；按从小到大顺序输出；-
            uint8_t signal_count;      // 占位4；该卫星跟踪频点数量；-
            double apd_ms;             // 占位18；概略距离使用时间表示，占 18 位，分为整毫秒数和毫秒余数，时间*光速恢复完整的概略距离。；ms
            int16_t approx_phase_rate; // 占位14；完整的相位距离变化率观测值可通过概略相位距离变化率（对卫星而言取值唯一）与精确相位距离变化率（对卫星信号而言取值唯一）相加得到。；m/s
            // Rsv 保留6bit
            struct
            {
                uint8_t signal_id;         // 占位6；频点标志，参见表 3-6；-
                uint8_t phase_lock_flag;   // 占位4；相位锁定时间标志与相位锁定时间对应关系参见表 3-7；-
                double precise_pr;         // 占位15；伪距的精确值；ms
                double precise_phase;      // 占位22；相位距离的精确值；ms
                double precise_phase_rate; // 占位15；精确相位距离变化率；m/s
                double cn0;                // 占位13；例：值 4295 表示载噪比为 42.95；dB
                uint8_t half_cycle;        // 占位1；表示是否使用的半周模糊度：0=没有半周模糊度 1=半周模糊度；-
                                           // Rsv 保留4bit
            } signal_info[PRANGEB_MAX_SATS];
        } sat_info[PRANGEB_MAX_SATS];
        uint32_t crc24; // 占位24；校验范围从帧头至有效数据；-
    } PRANGEB_Decoded_t;

    typedef struct
    {
        uint8_t en;
        int fd;
        char path[128];
    } File_cfg_t;
    
    typedef struct
    {
        File_cfg_t gpsephb;
        File_cfg_t bd2ephb;
        File_cfg_t bd3ephb;
        File_cfg_t gloephb;
        File_cfg_t galephb;
        File_cfg_t bdxwephb;
        File_cfg_t bd3cnav2ephb;
        File_cfg_t bd3cnav3ephb;
    } EPHB_File_sw_t;

    void decode_gpsephb(const uint8_t *payload, size_t payload_len, GPSEPHB_Decoded_t *out);
    void decode_bd2ephb(const uint8_t *payload, size_t payload_len, BD2EPHB_Decoded_t *out);
    void decode_bd3ephb(const uint8_t *payload, size_t payload_len, BD3EPHB_Decoded_t *out);
    void decode_gloephb(const uint8_t *payload, size_t payload_len, GLOEPHB_Decoded_t *out);
    void decode_galephb(const uint8_t *payload, size_t payload_len, GALEPHB_Decoded_t *out);
    void decode_bd3cnav2ephb(const uint8_t *payload, size_t payload_len, BD3CNAV2EPHB_Decoded_t *out);
    void decode_bd3cnav3ephb(const uint8_t *payload, size_t payload_len, BD3CNAV3EPHB_Decoded_t *out);
    void decode_posdatab(const uint8_t *payload, size_t payload_len, POSDATAB_Decoded_t *out);
    void decode_bdxwephb(const uint8_t *payload, size_t payload_len, BDXWEPHB_Decoded_t *out);
    void decode_prangeb(const uint8_t *payload, size_t payload_len, PRANGEB_Decoded_t *out);

    void handle_gnss_nmea(const char *sentence);
    int handle_gnss_raw(const uint8_t *data, size_t len);
    void gnss_cfg_disable_all(int fd);
    void gnss_cfg_enable_onchange(int fd, char *type);
    void gnss_cfg_dis_enable(int fd, char *type, uint8_t enable, uint8_t per_second);
    void gnss_cfg_mode(int fd, char *workMode, char *calcType, uint8_t freqCode);
    int8_t gnss_bdd_enable();
    int8_t gnss_bdd_disable();

    void gpsephb_file_header();
    void gpsephb_file_header2();

    char *gps_week_sec_to_utc(uint16_t gps_week, uint32_t gps_sec);

    void print_bd2ephb(const BD2EPHB_Decoded_t *eph, uint32_t payload_crc_calc);
    void print_bd3ephb(const BD3EPHB_Decoded_t *eph, uint32_t payload_crc_calc);
    void print_bd3cnav2ephb(const BD3CNAV2EPHB_Decoded_t *eph, uint32_t payload_crc_calc);
    void print_bd3cnav3ephb(const BD3CNAV3EPHB_Decoded_t *eph, uint32_t payload_crc_calc);
    void print_bdxwephb(const BDXWEPHB_Decoded_t *eph, uint32_t payload_crc_calc);
    void print_gpsephb(const GPSEPHB_Decoded_t *eph, uint32_t payload_crc_calc);
    void print_gpsephb_simple(const GPSEPHB_Decoded_t *eph);
    void print_gloephb(const GLOEPHB_Decoded_t *eph, uint32_t payload_crc_calc);
    void print_galephb(const GALEPHB_Decoded_t *eph, uint32_t payload_crc_calc);
    void print_posdatab(const POSDATAB_Decoded_t *pos, uint32_t payload_crc_calc);
    void print_prangeb(const PRANGEB_Decoded_t *prange, uint32_t payload_crc_calc);

    char *gnss_raw_info_file_header(char *type, uint8_t enable);
    void bd2ephb_file_append(const BD2EPHB_Decoded_t *eph);
    void bd3ephb_file_append(const BD3EPHB_Decoded_t *eph);
    void bd3cnav2ephb_file_append(const BD3CNAV2EPHB_Decoded_t *eph);
    void bd3cnav3ephb_file_append(const BD3CNAV3EPHB_Decoded_t *eph);
    void bdxwephb_file_append(const BDXWEPHB_Decoded_t *eph);
    void gpsephb_file_append(const GPSEPHB_Decoded_t *eph);

    extern gnss_ctrl_t gnss_ctrl;
    extern EPHB_File_sw_t ephb_file_sw;

#ifdef __cplusplus
}
#endif
