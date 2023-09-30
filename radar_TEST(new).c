/**************************************************************************
*
* 说明：该程序为基于飞腾平台的远距离探测系统
* 作者：彭勇智
* 时间：2023年9月22日
*
**************************************************************************/

#include "vsip.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>


/**************函数返回值的定义开始**************/
#define TWD_OK                     ( 0)
#define TWD_ERROR		           (-1)
#define TWD_EPARAM                 (-2)
#define TWD_FILE_OPEN_FAILED       (-3)
#define TWD_FILE_READ_ERROE        (-4)
#define TWD_FILE_WRITE_ERROE       (-5)
#define TWD_MEMORY_ALLOCATE_FAILED (-6)
#define TWD_FILE_NO_LOAD           (-7)
/**************函数返回值的定义结束**************/
#ifndef NULL
#define NULL  ((void *)0)
#endif
/**************雷达宏参数的定义开始**************/
#define TAO		7e-6			// 脉冲宽度
#define FS		20e6			// 采样率
#define FL		222e6			// 下限频率
#define BW		6e6				// 带宽
#define CF      225e6;          // 中心频率
#define OB      600;            // 物体距离
#define N       10;             // 希尔伯特滤波器阶数
#define PI		VSIP_PI
#define REPEAT	1
/**************雷达宏参数的定义结束**************/


/****************************内部接口申明开始****************************/

/*
 * 内部接口：initialize
 * 参数：n_signal_len--线性调频信号的长度
 * 			n_fft_len--FFT的长度
 * 功能：初始化VSIPL
 *			为各个向量分配空间
 *			初始化FFT与IFFT的变换参数
 * 返回值：TWD_OK--成功
 *			TWD_ERROR--失败
 *			TWD_EPARAM--参数错误
 */
int initialize(vsip_length n_signal_len, vsip_length n_fft_len);


/*
 * 内部接口：finalize
 * 参数：无
 * 功能：销毁各个向量
 *			销毁FFT与IFFT的参数对象
 *			结束VSIPL
 * 返回值：TWD_OK--成功
 *			TWD_ERROR--失败
 *			TWD_EPARAM--参数错误
 */
int finalize(void);


/*
 * 内部接口：init_lfm_signal
 * 参数：无
 * 功能：初始化线性调频信号
 * 返回值：TWD_OK--成功
 *			TWD_ERROR--失败
 *			TWD_EPARAM--参数错误
 */
int init_lfm_signal(void);


/*
 * 内部接口：tw_cvfliplr_f
 * 参数：a--需要被翻褶的向量视图指针
 * 			r--翻褶后的结果向量视图指针
 * 功能：将一个复向量翻褶
 * 返回值：TWD_OK--成功
 *			TWD_ERROR--失败
 *			TWD_EPARAM--参数错误
 */
int tw_cvfliplr_f(vsip_cvview_f* a, vsip_cvview_f* r);


/*
 * 内部接口：init_match_filter
 * 参数：无
 * 功能：初始化匹配滤波器向量
 * 返回值：TWD_OK--成功
 *			TWD_ERROR--失败
 *			TWD_EPARAM--参数错误
 */
int init_match_filter(void);


/*
 * 内部接口：tw_cvaddwin_f
 * 参数：a--需要被加窗的向量视图指针
 * 			r--加窗后的结果向量视图指针
 * 功能：对复向量加窗函数
 * 返回值：TWD_OK--成功
 *			TWD_ERROR--失败
 *			TWD_EPARAM--参数错误
 */
int tw_cvaddwin_f(vsip_cvview_f* a, vsip_cvview_f* r);


/*
 * 内部接口：tw_cvaddzero_f
 * 参数：a--需要补零的向量视图指针
 * 			r--补零后的结果向量视图指针
 * 功能：对复向量补零
 * 返回值：TWD_OK--成功
 *			TWD_ERROR--失败
 *			TWD_EPARAM--参数错误
 */
int tw_cvaddzero_f(vsip_cvview_f* a, vsip_cvview_f* r);


/*
 * 内部接口：tw_vprint_i
 * 参数：a--待打印的整数向量视图指针
 * 功能：串口打印一个整数向量
 * 返回值：TWD_OK--成功
 *			TWD_ERROR--失败
 *			TWD_EPARAM--参数错误
 */
int tw_vprint_i(vsip_vview_i* a);


/*
 * 内部接口：tw_vprint_vi
 * 参数：a--待打印的索引向量视图指针
 * 功能：串口打印一个索引向量
 * 返回值：TWD_OK--成功
 *			TWD_ERROR--失败
 *			TWD_EPARAM--参数错误
 */
int tw_vprint_vi(vsip_vview_vi* a);


/*
 * 内部接口：tw_vprint_f
 * 参数：a--待打印的实向量视图指针
 * 功能：串口打印一个实向量
 * 返回值：TWD_OK--成功
 *			TWD_ERROR--失败
 *			TWD_EPARAM--参数错误
 */
int tw_vprint_f(vsip_vview_f* a);


/*
 * 内部接口：tw_cvprint_f
 * 参数：a--待打印的复向量视图指针
 * 功能：串口打印一个复向量
 * 返回值：TWD_OK--成功
 *			TWD_ERROR--失败
 *			TWD_EPARAM--参数错误
 */
int tw_cvprint_f(vsip_cvview_f* a);

/****************************内部接口申明结束****************************/






/**************************全局变量定义开始**************************/

vsip_vview_f *g_p_vector_time = NULL;										// 时间轴

vsip_cvview_f *g_p_vector_lfm_signal = NULL;								// 线性调频信号
vsip_cvview_f *g_p_vector_lfm_signal_addzero = NULL;						// 线性调频信号、补零
vsip_cvview_f *g_p_vector_lfm_signal_addzero_fft = NULL;					// 线性调频信号、补零、FFT

vsip_cvview_f *g_p_vector_match_filter = NULL;								// 匹配滤波器
vsip_cvview_f *g_p_vector_match_filter_addwindows = NULL;					// 匹配滤波器、加窗
vsip_cvview_f *g_p_vector_match_filter_addwindows_addzero = NULL;			// 匹配滤波器、加窗、补零
vsip_cvview_f *g_p_vector_match_filter_addwindows_addzero_fft = NULL;		// 匹配滤波器、加窗、补零、FFT

vsip_cvview_f *g_p_vector_match_filter_out = NULL;							// 脉冲压缩信号
vsip_cvview_f *g_p_vector_match_filter_out_fft = NULL;						// 脉冲压缩信号、FFT


vsip_fft_f *g_p_fft_op_plan = NULL;											// FFT参数
vsip_fft_f *g_p_ifft_op_plan = NULL;										// IFFT参数

struct timeval g_st_timer_begin, g_st_timer_end;							// 计时器
double g_lf_delta_timer = 0.0, g_lf_timer_sum = 0.0;						// 计时器差值与计时累加器

/**************************全局变量定义结束**************************/


int main(int argc,char *argv[])
{
	vsip_scalar_f f_power = 0.0;									// 信号在某一点的功率值
	vsip_scalar_f f_power_dB = 0.0;									// 信号在某一点的功率值（dB）
	vsip_scalar_vi n_index = 0;										// 该功率值对应的索引号
	vsip_length n_signal_len = (vsip_length)(0.5 + TAO * FS);		// 求出信号的点数
	vsip_length n_fft_len = n_signal_len + n_signal_len - 1;		// 循环卷积等于线性卷积，n_fft_len有最小值，并求之
	n_fft_len = (vsip_length)(ceil)( 0.5 + log2(n_fft_len) );		// FFT要求n_fft_len为2的整数次幂
	n_fft_len = (vsip_length)(floor)( 0.5 + pow(2, n_fft_len) );	// 最终求得FFT点数
	printf("\n\n---------------------------------------start---------------------------------------------\n");
	printf("n_signal_len = %ld, n_fft_len = %ld\n", n_signal_len, n_fft_len);
	
	
	// VSIPL的初始化、向量视图对象内存分配、FFT与IFFT对象初始化
	initialize(n_signal_len, n_fft_len);
	printf("initialize has been run\n\n");
	
	
for(int k = 0; k < REPEAT; k++)		// 重复实验
{
	// 计算频率调制率 
    float modulation_rate = BW / TAO;
	
	// 线性调频信号的初始化
	gettimeofday(&g_st_timer_begin, NULL);
	
	init_lfm_signal();	
	/*
	vsip_vramp_f(FL, modulation_rate / 2.0, g_p_vector_lfm_signal);
    vsip_vcos_f(g_p_vector_lfm_signal, g_p_vector_lfm_signal);
	*/

	gettimeofday(&g_st_timer_end, NULL);
	g_lf_delta_timer = (double)(g_st_timer_end.tv_sec - g_st_timer_begin.tv_sec) * 1E6
						+ (double)(g_st_timer_end.tv_usec - g_st_timer_begin.tv_usec);
	g_lf_timer_sum = g_lf_timer_sum + g_lf_delta_timer;
	printf("delta_timer : %.6f us\n", g_lf_delta_timer);
	printf("timer_sum : %.6f us\n", g_lf_timer_sum);
	printf("vector_lfm_signal:\n");
	tw_cvprint_f(g_p_vector_lfm_signal);
	printf("\n");

	// 回波信号
	// 计算时间延迟
    float delay = OB * 2.0 / 3.0e8;

    // 创建第一个物体的回波信号
    vsip_vview_f *echo1 = vsip_vcreate_f(n_signal_len, VSIP_MEM_NONE);
    vsip_vputlength_f(g_p_vector_lfm_signal, (int)(TAO * FS - delay * FS));
    vsip_vputoffset_f(g_p_vector_lfm_signal, (int)(delay * FS));
    vsip_vcopy_f_f(g_p_vector_lfm_signal, echo1);

    // 创建第二个物体的回波信号
    vsip_vview_f *echo2 = vsip_vcreate_f(n_signal_len, VSIP_MEM_NONE);
    vsip_vputlength_f(g_p_vector_lfm_signal, (int)(TAO * FS - (delay + OB / 3.0e8) * FS));
    vsip_vputoffset_f(g_p_vector_lfm_signal, (int)((delay + OB / 3.0e8) * FS));
    vsip_vcopy_f_f(g_p_vector_lfm_signal, echo2);

    // 叠加两个回波信号
    vsip_vadd_f(echo1, echo2, echo1);

	// 创建带通滤波器 
    int filter_length = (int)(FS / BW);
    vsip_vview_f *bp_filter = vsip_vcreate_hanning_f(filter_length, VSIP_MEM_NONE);
    vsip_rcvadd_f(bp_filter, CF, bp_filter);

    // 进行卷积运算 
    int output_length = (int)((TAO + delay + object_distance / 3.0e8) * FS);
    vsip_conv1d_f(vsip_vcreate_f(output_length, VSIP_MEM_NONE), VSIP_CONVolve, bp_filter, echo1, output_length - filter_length + 1, filter_length);

	// 添加高斯白噪声
	// 创建噪声向量
    vsip_vview_f *noise = vsip_vcreate_f(n_signal_len, 0);

    // 计算信号功率
    float signal_power = vsip_vdot_f(echo1, echo1);

    // 计算噪声功率，以使SNR为0 dB
    float snr_dB = 0.0;
    float snr_linear = pow(10.0, snr_dB / 10.0); // 将dB转换为线性值
    float noise_power = signal_power / snr_linear;

    // 生成高斯白噪声
    vsip_vrandn_f(noise); // 生成高斯白噪声
    vsip_vsmsa_f(noise, sqrt(noise_power), noise, 0.0, noise); // 设置噪声的标准差和均值

    // 将噪声添加到信号中
    vsip_vadd_f(echo1, noise, echo1);
	

	// 希尔伯特滤波器
	// 创建希尔伯特滤波器
    vsip_vview_f *hilbert = vsip_vcreate_f(N, VSIP_MEM_NONE);
    vsip_vramp_f(1.0, -2.0 / (float)N, hilbert);

    // 执行时域滤波 
    vsip_vview_f *hilbert_output = vsip_vcreate_f(N, VSIP_MEM_NONE);
    vsip_vconv_f(hilbert, echo1, hilbert_output);


	// 匹配滤波器的初始化
	gettimeofday(&g_st_timer_begin, NULL);
	init_match_filter();
	gettimeofday(&g_st_timer_end, NULL);
	g_lf_delta_timer = (double)(g_st_timer_end.tv_sec - g_st_timer_begin.tv_sec) * 1E6
						+ (double)(g_st_timer_end.tv_usec - g_st_timer_begin.tv_usec);
	g_lf_timer_sum = g_lf_timer_sum + g_lf_delta_timer;
	printf("delta_timer : %.6f us\n", g_lf_delta_timer);
	printf("timer_sum : %.6f us\n", g_lf_timer_sum);
	printf("vector_match_filter:\n");
	tw_cvprint_f(g_p_vector_match_filter);
	printf("\n");
	
	
	// 匹配滤波器加窗函数
	gettimeofday(&g_st_timer_begin, NULL);
	tw_cvaddwin_f(g_p_vector_match_filter, g_p_vector_match_filter_addwindows);
	gettimeofday(&g_st_timer_end, NULL);
	g_lf_delta_timer = (double)(g_st_timer_end.tv_sec - g_st_timer_begin.tv_sec) * 1E6
						+ (double)(g_st_timer_end.tv_usec - g_st_timer_begin.tv_usec);
	g_lf_timer_sum = g_lf_timer_sum + g_lf_delta_timer;
	printf("delta_timer : %.6f us\n", g_lf_delta_timer);
	printf("timer_sum : %.6f us\n", g_lf_timer_sum);
	printf("vector_match_filter_addwindows:\n");
	tw_cvprint_f(g_p_vector_match_filter_addwindows);
	printf("\n");
	
	
	// 线性调频信号的补零操作
	// 信号补零
	gettimeofday(&g_st_timer_begin, NULL);
	//tw_cvaddzero_f(g_p_vector_lfm_signal, g_p_vector_lfm_signal_addzero);
	tw_cvaddzero_f(hilbert_output, g_p_vector_lfm_signal_addzero);
	gettimeofday(&g_st_timer_end, NULL);
	g_lf_delta_timer = (double)(g_st_timer_end.tv_sec - g_st_timer_begin.tv_sec) * 1E6
						+ (double)(g_st_timer_end.tv_usec - g_st_timer_begin.tv_usec);
	g_lf_timer_sum = g_lf_timer_sum + g_lf_delta_timer;
	printf("delta_timer : %.6f us\n", g_lf_delta_timer);
	printf("timer_sum : %.6f us\n", g_lf_timer_sum);
	printf("vector_lfm_signal_addzero:\n");
	tw_cvprint_f(g_p_vector_lfm_signal_addzero);
	printf("\n");
	
	
	// 加窗后的匹配滤波器的补零操作
	gettimeofday(&g_st_timer_begin, NULL);
	tw_cvaddzero_f(g_p_vector_match_filter_addwindows, g_p_vector_match_filter_addwindows_addzero);
	gettimeofday(&g_st_timer_end, NULL);
	g_lf_delta_timer = (double)(g_st_timer_end.tv_sec - g_st_timer_begin.tv_sec) * 1E6
						+ (double)(g_st_timer_end.tv_usec - g_st_timer_begin.tv_usec);
	g_lf_timer_sum = g_lf_timer_sum + g_lf_delta_timer;
	printf("delta_timer : %.6f us\n", g_lf_delta_timer);
	printf("timer_sum : %.6f us\n", g_lf_timer_sum);
	printf("vector_match_filter_addwindows_addzero:\n");
	tw_cvprint_f(g_p_vector_match_filter_addwindows_addzero);
	printf("\n");
	
	
	// 线性调频信号的FFT
	gettimeofday(&g_st_timer_begin, NULL);
	vsip_ccfftop_f(g_p_fft_op_plan, g_p_vector_lfm_signal_addzero, g_p_vector_lfm_signal_addzero_fft);
	gettimeofday(&g_st_timer_end, NULL);
	g_lf_delta_timer = (double)(g_st_timer_end.tv_sec - g_st_timer_begin.tv_sec) * 1E6
						+ (double)(g_st_timer_end.tv_usec - g_st_timer_begin.tv_usec);
	g_lf_timer_sum = g_lf_timer_sum + g_lf_delta_timer;
	printf("delta_timer : %.6f us\n", g_lf_delta_timer);
	printf("timer_sum : %.6f us\n", g_lf_timer_sum);
	printf("vector_lfm_signal_addzero_fft:\n");
	tw_cvprint_f(g_p_vector_lfm_signal_addzero_fft);
	printf("\n");
	
	
	// 匹配滤波器的FFT
	gettimeofday(&g_st_timer_begin, NULL);
	vsip_ccfftop_f(g_p_fft_op_plan, g_p_vector_match_filter_addwindows_addzero, g_p_vector_match_filter_addwindows_addzero_fft);
	gettimeofday(&g_st_timer_end, NULL);
	g_lf_delta_timer = (double)(g_st_timer_end.tv_sec - g_st_timer_begin.tv_sec) * 1E6
						+ (double)(g_st_timer_end.tv_usec - g_st_timer_begin.tv_usec);
	g_lf_timer_sum = g_lf_timer_sum + g_lf_delta_timer;
	printf("delta_timer : %.6f us\n", g_lf_delta_timer);
	printf("timer_sum : %.6f us\n", g_lf_timer_sum);
	printf("vector_match_filter_addwindows_addzero_fft:\n");
	tw_cvprint_f(g_p_vector_match_filter_addwindows_addzero_fft);
	printf("\n");
	
	
	// 频域相乘
	gettimeofday(&g_st_timer_begin, NULL);
	vsip_cvmul_f(g_p_vector_lfm_signal_addzero_fft, g_p_vector_match_filter_addwindows_addzero_fft, g_p_vector_match_filter_out_fft);
	gettimeofday(&g_st_timer_end, NULL);
	g_lf_delta_timer = (double)(g_st_timer_end.tv_sec - g_st_timer_begin.tv_sec) * 1E6
						+ (double)(g_st_timer_end.tv_usec - g_st_timer_begin.tv_usec);
	g_lf_timer_sum = g_lf_timer_sum + g_lf_delta_timer;
	printf("delta_timer : %.6f us\n", g_lf_delta_timer);
	printf("timer_sum : %.6f us\n", g_lf_timer_sum);
	printf("vector_match_filter_out_fft:\n");
	tw_cvprint_f(g_p_vector_match_filter_out_fft);
	printf("\n");
	
	
	// 匹配滤波器输出信号的频域变时域
	gettimeofday(&g_st_timer_begin, NULL);
	vsip_ccfftop_f(g_p_ifft_op_plan, g_p_vector_match_filter_out_fft, g_p_vector_match_filter_out);
	gettimeofday(&g_st_timer_end, NULL);
	g_lf_delta_timer = (double)(g_st_timer_end.tv_sec - g_st_timer_begin.tv_sec) * 1E6
						+ (double)(g_st_timer_end.tv_usec - g_st_timer_begin.tv_usec);
	g_lf_timer_sum = g_lf_timer_sum + g_lf_delta_timer;
	printf("delta_timer : %.6f us\n", g_lf_delta_timer);
	printf("timer_sum : %.6f us\n", g_lf_timer_sum);
	printf("vector_match_filter_out:\n");
	tw_cvprint_f(g_p_vector_match_filter_out);
	printf("\n");

	// 雷达目标探测
	// 执行通用化的目标检测
    int target_count = 0;
    int target1_position = -1; // 记录第一个目标的位置
    int target2_position = -1; // 记录第二个目标的位置

    // 假设这里使用一个简单的门限检测算法来检测目标
    float detection_threshold = 0.5; // 门限值

    // 遍历脉冲压缩后的信号
    for (int i = 0; i < n_signal_len; i++) {
        float amplitude = vsip_vget_f(g_p_vector_match_filter_out, i);

        // 使用门限值进行检测
        if (amplitude > detection_threshold) {
            target_count++;

            // 如果检测到第一个目标
            if (target_count == 1) {
                target1_position = i;
            } else if (target_count == 2) { // 如果检测到第二个目标
                target2_position = i;
                break; // 已检测到两个目标，不再继续检测
            }
        }
    }

    // 计算两个目标的距离
    if (target1_position >= 0 && target2_position >= 0) {
        // 计算目标1的到达时间延迟
        double target1_delay = (double)target1_position / FS;

        // 计算目标2的到达时间延迟
        double target2_delay = (double)target2_position / FS;

        // 计算两个目标的距离
        double distance_meters = fabs(target2_delay - target1_delay) * 3e8 / 2.0;

        printf("已检测到两个目标，目标距离为：%lf 米\n", distance_meters);
    } 
	else {
        printf("未检测到足够的目标\n");
    }

	
}
g_lf_timer_sum = g_lf_timer_sum / REPEAT;
	
	
	// 前置算法所消耗的时间
	printf("Preposition algorithm takes %.6f us\n", g_lf_timer_sum);
	
	
	// 求最高点的功率以及索引号
	gettimeofday(&g_st_timer_begin, NULL);
	f_power = vsip_vcmaxmgsqval_f(g_p_vector_match_filter_out, &n_index);
	f_power_dB = 10*log10(f_power);
	gettimeofday(&g_st_timer_end, NULL);
	g_lf_delta_timer = (double)(g_st_timer_end.tv_sec - g_st_timer_begin.tv_sec) * 1E6
						+ (double)(g_st_timer_end.tv_usec - g_st_timer_begin.tv_usec);
	g_lf_timer_sum = g_lf_timer_sum + g_lf_delta_timer;
	printf("delta_timer : %.6f us\n", g_lf_delta_timer);
	printf("timer_sum : %.6f us\n", g_lf_timer_sum);
	printf("max power = %f = %fdB\n", f_power, f_power_dB);
	printf("p_index = %p = 0x%lx\n", &n_index, (unsigned long int)&n_index);
	printf("max power index = %ld\n", n_index);
	
	
	// 销毁各个向量视图、销毁FFT与IFFT的参数对象、结束VSIPL
	printf("\n");
	gettimeofday(&g_st_timer_begin, NULL);
	finalize();
	gettimeofday(&g_st_timer_end, NULL);
	g_lf_delta_timer = (double)(g_st_timer_end.tv_sec - g_st_timer_begin.tv_sec) * 1E6
						+ (double)(g_st_timer_end.tv_usec - g_st_timer_begin.tv_usec);
	g_lf_timer_sum = g_lf_timer_sum + g_lf_delta_timer;
	printf("delta_timer : %.6f us\n", g_lf_delta_timer);
	printf("timer_sum : %.6f us\n", g_lf_timer_sum);
	printf("finalize has been run\n\n");
	printf("\n----------------------------------------end---------------------------------------------\n\n");
	return 0;
}



/****************************内部接口定义开始****************************/

/*
 * 内部接口：initialize
 * 参数：n_signal_len--线性调频信号的长度
 * 			n_fft_len--FFT的长度
 * 功能：初始化VSIPL
 *			为各个向量分配空间
 *			初始化FFT与IFFT的变换参数
 * 返回值：TWD_OK--成功
 *			TWD_ERROR--失败
 *			TWD_EPARAM--参数错误
 */
int initialize(vsip_length n_signal_len, vsip_length n_fft_len)
{
    int s = vsip_init(NULL);
	if(s!=0)
	{
		printf("initialize: Err, vsip_init failed!\n\n");
		return TWD_ERROR;
	}
	else
	{
		;
	}
	
	g_p_vector_time = vsip_vcreate_f(n_signal_len, VSIP_MEM_NONE);
	if(g_p_vector_time==NULL)
	{
		printf("initialize: Err, g_p_vector_time==NULL!\n\n");
		return TWD_ERROR;
	}
	else
	{
		;
	}
	
    g_p_vector_lfm_signal = vsip_cvcreate_f(n_signal_len, VSIP_MEM_NONE);
	if(g_p_vector_lfm_signal==NULL)
	{
		printf("initialize: Err, g_p_vector_lfm_signal==NULL!\n\n");
		return TWD_ERROR;
	}
	else
	{
		;
	}
	
	g_p_vector_lfm_signal_addzero = vsip_cvcreate_f(n_fft_len, VSIP_MEM_NONE);
	if(g_p_vector_lfm_signal_addzero==NULL)
	{
		printf("initialize: Err, g_p_vector_lfm_signal_addzero==NULL!\n\n");
		return TWD_ERROR;
	}
	else
	{
		;
	}
	
	g_p_vector_lfm_signal_addzero_fft = vsip_cvcreate_f(n_fft_len, VSIP_MEM_NONE);
	if(g_p_vector_lfm_signal_addzero_fft==NULL)
	{
		printf("initialize: Err, g_p_vector_lfm_signal_addzero_fft==NULL!\n\n");
		return TWD_ERROR;
	}
	else
	{
		;
	}
	
	g_p_vector_match_filter = vsip_cvcreate_f(n_signal_len, VSIP_MEM_NONE);
	if(g_p_vector_match_filter==NULL)
	{
		printf("initialize: Err, g_p_vector_match_filter==NULL!\n\n");
		return TWD_ERROR;
	}
	else
	{
		;
	}
	
	g_p_vector_match_filter_addwindows = vsip_cvcreate_f(n_signal_len, VSIP_MEM_NONE);
	if(g_p_vector_match_filter_addwindows==NULL)
	{
		printf("initialize: Err, g_p_vector_match_filter_addwindows==NULL!\n\n");
		return TWD_ERROR;
	}
	else
	{
		;
	}
	
	g_p_vector_match_filter_addwindows_addzero = vsip_cvcreate_f(n_fft_len, VSIP_MEM_NONE);
	if(g_p_vector_match_filter_addwindows_addzero==NULL)
	{
		printf("initialize: Err, g_p_vector_match_filter_addwindows_addzero==NULL!\n\n");
		return TWD_ERROR;
	}
	else
	{
		;
	}
	
	g_p_vector_match_filter_addwindows_addzero_fft = vsip_cvcreate_f(n_fft_len, VSIP_MEM_NONE);
	if(g_p_vector_match_filter_addwindows_addzero_fft==NULL)
	{
		printf("initialize: Err, g_p_vector_match_filter_addwindows_addzero_fft==NULL!\n\n");
		return TWD_ERROR;
	}
	else
	{
		;
	}
	
	g_p_vector_match_filter_out = vsip_cvcreate_f(n_fft_len, VSIP_MEM_NONE);
	if(g_p_vector_match_filter_out==NULL)
	{
		printf("initialize: Err, g_p_vector_match_filter_out==NULL!\n\n");
		return TWD_ERROR;
	}
	else
	{
		;
	}
	
	g_p_vector_match_filter_out_fft = vsip_cvcreate_f(n_fft_len, VSIP_MEM_NONE);
	if(g_p_vector_match_filter_out_fft==NULL)
	{
		printf("initialize: Err, g_p_vector_match_filter_out_fft==NULL!\n\n");
		return TWD_ERROR;
	}
	else
	{
		;
	}
	
    //初始化FFT正变换参数
    g_p_fft_op_plan = vsip_ccfftop_create_f(n_fft_len, 1.0, VSIP_FFT_FWD, 1, 0);
	if(g_p_fft_op_plan==NULL)
	{
		printf("initialize: Err, g_p_fft_op_plan==NULL!\n\n");
		return TWD_ERROR;
	}
	else
	{
		;
	}
	
    //初始化FFT逆变换参数
    g_p_ifft_op_plan= vsip_ccfftop_create_f(n_fft_len, 1.0/n_fft_len, VSIP_FFT_INV, 1, 0);
	if(g_p_ifft_op_plan==NULL)
	{
		printf("initialize: Err, g_p_ifft_op_plan==NULL!\n\n");
		return TWD_ERROR;
	}
	else
	{
		;
	}
	
	return TWD_OK;
}



/*
 * 内部接口：finalize
 * 参数：无
 * 功能：销毁各个向量
 *			销毁FFT与IFFT的参数对象
 *			结束VSIPL
 * 返回值：TWD_OK--成功
 *			TWD_ERROR--失败
 *			TWD_EPARAM--参数错误
 */
int finalize(void)
{
    vsip_valldestroy_f(g_p_vector_time);
	g_p_vector_time = NULL;
    vsip_cvalldestroy_f(g_p_vector_lfm_signal);
	g_p_vector_lfm_signal = NULL;
    vsip_cvalldestroy_f(g_p_vector_lfm_signal_addzero);
	g_p_vector_lfm_signal_addzero = NULL;
	vsip_cvalldestroy_f(g_p_vector_lfm_signal_addzero_fft);
	g_p_vector_lfm_signal_addzero_fft = NULL;
	vsip_cvalldestroy_f(g_p_vector_match_filter);
	g_p_vector_match_filter = NULL;
	vsip_cvalldestroy_f(g_p_vector_match_filter_addwindows);
	g_p_vector_match_filter_addwindows = NULL;
	vsip_cvalldestroy_f(g_p_vector_match_filter_addwindows_addzero);
	g_p_vector_match_filter_addwindows_addzero = NULL;
	vsip_cvalldestroy_f(g_p_vector_match_filter_addwindows_addzero_fft);
	g_p_vector_match_filter_addwindows_addzero_fft = NULL;
	vsip_cvalldestroy_f(g_p_vector_match_filter_out);
	g_p_vector_match_filter_out = NULL;
	vsip_cvalldestroy_f(g_p_vector_match_filter_out_fft);
	g_p_vector_match_filter_out_fft = NULL;
	
    int s = vsip_fft_destroy_f(g_p_fft_op_plan);
	if(s!=0)
	{
		printf("finalize: Err, g_p_fft_op_plan destroy failed!\n\n");
		return TWD_ERROR;
	}
	else
	{
		g_p_fft_op_plan = NULL;
	}
	
    s = vsip_fft_destroy_f(g_p_ifft_op_plan);
	if(s!=0)
	{
		printf("finalize: Err, g_p_ifft_op_plan destroy failed!\n\n");
		return TWD_ERROR;
	}
	else
	{
		g_p_ifft_op_plan = NULL;
	}
    
    s = vsip_finalize((void *)0);
	if(s!=0)
	{
		printf("finalize: Err, vsip_finalize failed!\n\n");
		return TWD_ERROR;
	}
	else
	{
		;
	}
	
	return TWD_OK;
}



/*
 * 内部接口：init_lfm_signal
 * 参数：无
 * 功能：初始化线性调频信号
 * 返回值：TWD_OK--成功
 *			TWD_ERROR--失败
 *			TWD_EPARAM--参数错误
 */
int init_lfm_signal(void)
{
	vsip_length n = vsip_cvgetlength_f(g_p_vector_lfm_signal);
	vsip_vview_f *p_vector_temp1 = NULL;
	p_vector_temp1 = vsip_vcreate_f(n, VSIP_MEM_NONE);
	if(p_vector_temp1==NULL)
	{
		printf("init_lfm_signal: Err, p_vector_temp1==NULL!\n\n");
		return TWD_ERROR;
	}
	else
	{
		;
	}
	
	vsip_vview_f *p_vector_temp2 = NULL;
	p_vector_temp2 = vsip_vcreate_f(n, VSIP_MEM_NONE);
	if(p_vector_temp2==NULL)
	{
		printf("init_lfm_signal: Err, p_vector_temp2==NULL!\n\n");
		return TWD_ERROR;
	}
	else
	{
		;
	}
	
	vsip_vview_f *p_vector_temp3 = NULL;
	p_vector_temp3 = vsip_vcreate_f(n, VSIP_MEM_NONE);
	if(p_vector_temp3==NULL)
	{
		printf("init_lfm_signal: Err, p_vector_temp3==NULL!\n\n");
		return TWD_ERROR;
	}
	else
	{
		;
	}
	
	vsip_vramp_f(0, 1.0/FS, g_p_vector_time);
	vsip_svmul_f(2*PI*FL, g_p_vector_time, p_vector_temp1);
	vsip_vmul_f(g_p_vector_time, g_p_vector_time, p_vector_temp2);
	vsip_svmul_f(PI*BW/TAO, p_vector_temp2, p_vector_temp3);
	vsip_vadd_f(p_vector_temp1, p_vector_temp3, p_vector_temp2);
	//vsip_veuler_f(p_vector_temp2, g_p_vector_lfm_signal);
	vsip_vcos_f(p_vector_temp2, g_p_vector_lfm_signal); // 余弦
	
	vsip_valldestroy_f(p_vector_temp1);
	p_vector_temp1 = NULL;
	vsip_valldestroy_f(p_vector_temp2);
	p_vector_temp2 = NULL;
	vsip_valldestroy_f(p_vector_temp3);
	p_vector_temp3 = NULL;
	
	return TWD_OK;
}



/*
 * 内部接口：tw_cvfliplr_f
 * 参数：a--需要被翻褶的向量视图指针
 * 			r--翻褶后的结果向量视图指针
 * 功能：将一个复向量翻褶
 * 返回值：TWD_OK--成功
 *			TWD_ERROR--失败
 *			TWD_EPARAM--参数错误
 */
 #if 1
int tw_cvfliplr_f(vsip_cvview_f* a, vsip_cvview_f* r)
{
	if(a==NULL)
	{
		printf("tw_cvfliplr_f: Err, a==NULL!\n\n");
		return TWD_EPARAM;
	}
	else if(r==NULL)
	{
		printf("tw_cvfliplr_f: Err, r==NULL!\n\n");
		return TWD_EPARAM;
	}
	else
	{
		;
	}
	
	vsip_cscalar_f x;
	vsip_index i;
	vsip_length n = vsip_cvgetlength_f(a);
	for(i = 0; i < n; i++)
	{
		x = vsip_cvget_f(a, i);
		vsip_cvput_f(r, n-1-i, x);
	}
	
	return TWD_OK;
}
#else
int tw_cvfliplr_f(vsip_cvview_f* a, vsip_cvview_f* r)
{
	vsip_length n = vsip_cvgetlength_f(a);
	vsip_vview_vi* p_vector_index = vsip_vcreate_vi(n, VSIP_MEM_NONE);
	vsip_vview_i* p_vector_i = vsip_vcreate_i(n, VSIP_MEM_NONE);
	
	vsip_vramp_i(n-1, -1, p_vector_index);
	printf("vector_index:\n");tw_vprint_vi(p_vector_index);printf("\n");
	
	vsip_vramp_i(n-1, -1, p_vector_i);
	printf("vector_i:\n");tw_vprint_i(p_vector_i);printf("\n");
	
	vsip_cvgather_f(a, p_vector_i, r);
	
	vsip_valldestroy_vi(p_vector_index);
	vsip_valldestroy_i(p_vector_i);
	
	return 0;
}
#endif



/*
 * 内部接口：init_match_filter
 * 参数：无
 * 功能：初始化匹配滤波器向量
 * 返回值：TWD_OK--成功
 *			TWD_ERROR--失败
 *			TWD_EPARAM--参数错误
 */
int init_match_filter(void)
{
	vsip_length n = vsip_cvgetlength_f(g_p_vector_lfm_signal);
	vsip_cvview_f *p_vector_temp = NULL;
	p_vector_temp = vsip_cvcreate_f(n, VSIP_MEM_NONE);
	if(p_vector_temp==NULL)
	{
		printf("init_match_filter: Err, p_vector_temp==NULL!\n\n");
		return TWD_ERROR;
	}
	else
	{
		;
	}
	
	vsip_cvconj_f(g_p_vector_lfm_signal, p_vector_temp);
	tw_cvfliplr_f(p_vector_temp, g_p_vector_match_filter);
	
	vsip_cvalldestroy_f(p_vector_temp);
	p_vector_temp = NULL;
	
	return TWD_OK;
}



/*
 * 内部接口：tw_cvaddwin_f
 * 参数：a--需要被加窗的向量视图指针
 * 			r--加窗后的结果向量视图指针
 * 功能：对复向量加窗函数
 * 返回值：TWD_OK--成功
 *			TWD_ERROR--失败
 *			TWD_EPARAM--参数错误
 */
int tw_cvaddwin_f(vsip_cvview_f* a, vsip_cvview_f* r)
{
	if(a==NULL)
	{
		printf("tw_cvaddwin_f: Err, a==NULL!\n\n");
		return TWD_EPARAM;
	}
	else if(r==NULL)
	{
		printf("tw_cvaddwin_f: Err, r==NULL!\n\n");
		return TWD_EPARAM;
	}
	else
	{
		;
	}
	
	vsip_length n = vsip_cvgetlength_f(a);
	vsip_vview_f *p_vector_windows = NULL;
	p_vector_windows = vsip_vcreate_hamming_f(n, VSIP_MEM_NONE); //汉明窗
	if(p_vector_windows==NULL)
	{
		printf("tw_cvaddwin_f: Err, p_vector_windows==NULL!\n\n");
		return TWD_ERROR;
	}
	else
	{
		;
	}
	
//	printf("vector_windows:\n");
//	tw_vprint_f(p_vector_windows);
//	printf("\n");
	
	vsip_rcvmul_f(p_vector_windows, a, r);
	vsip_valldestroy_f(p_vector_windows);
	p_vector_windows = NULL;
	
	return TWD_OK;
}



/*
 * 内部接口：tw_cvaddzero_f
 * 参数：a--需要补零的向量视图指针
 * 			r--补零后的结果向量视图指针
 * 功能：对复向量补零
 * 返回值：TWD_OK--成功
 *			TWD_ERROR--失败
 *			TWD_EPARAM--参数错误
 */
int tw_cvaddzero_f(vsip_cvview_f* a, vsip_cvview_f* r)
{
	if(a==NULL)
	{
		printf("tw_cvaddzero_f: Err, a==NULL!\n\n");
		return TWD_EPARAM;
	}
	else if(r==NULL)
	{
		printf("tw_cvaddzero_f: Err, r==NULL!\n\n");
		return TWD_EPARAM;
	}
	else
	{
		;
	}
	
	vsip_cscalar_f x;
	vsip_length n = vsip_cvgetlength_f(a);
	vsip_length m = vsip_cvgetlength_f(r);
	vsip_cvview_f *p_vector_zero = NULL;
	p_vector_zero = vsip_cvcreate_f(m-n, VSIP_MEM_NONE);
	if(p_vector_zero==NULL)
	{
		printf("tw_cvaddzero_f: Err, p_vector_zero==NULL!\n\n");
		return TWD_ERROR;
	}
	else
	{
		;
	}
	
	x.r = 0.0;
	x.i = 0.0;
	vsip_cvfill_f(x, p_vector_zero);
	vsip_cvconcatenate_f(a, p_vector_zero, r);
	vsip_cvalldestroy_f(p_vector_zero);
	p_vector_zero = NULL;
	
	return TWD_OK;
}



/*
 * 内部接口：tw_vprint_i
 * 参数：a--待打印的整数向量视图指针
 * 功能：串口打印一个整数向量
 * 返回值：TWD_OK--成功
 *			TWD_ERROR--失败
 *			TWD_EPARAM--参数错误
 */
int tw_vprint_i(vsip_vview_i* a)
{
	if(a==NULL)
	{
		printf("tw_vprint_i: Err, a==NULL!\n\n");
		return TWD_EPARAM;
	}
	else
	{
		;
	}
	
	vsip_length i, n;
	vsip_scalar_i x;
	n = vsip_vgetlength_i(a);
	for(i=0; i<n; i++)
	{
		x = vsip_vget_i(a,i);
		printf("%d\t", x);
		if( (i + 1) % 8 == 0 )
		{
			printf("\n");
		}
	}
	printf("\n");
	
	return TWD_OK;
}



/*
 * 内部接口：tw_vprint_vi
 * 参数：a--待打印的索引向量视图指针
 * 功能：串口打印一个索引向量
 * 返回值：TWD_OK--成功
 *			TWD_ERROR--失败
 *			TWD_EPARAM--参数错误
 */
int tw_vprint_vi(vsip_vview_vi* a)
{
	if(a==NULL)
	{
		printf("tw_vprint_vi: Err, a==NULL!\n\n");
		return TWD_EPARAM;
	}
	else
	{
		;
	}
	
	vsip_length i, n;
	vsip_scalar_vi x;
	n = vsip_vgetlength_vi(a);
	for(i=0; i<n; i++)
	{
		x = vsip_vget_vi(a,i);
		printf("%ld\t", x);
		if( (i + 1) % 8 == 0 )
		{
			printf("\n");
		}
	}
	printf("\n");
	
	return TWD_OK;
}



/*
 * 内部接口：tw_vprint_f
 * 参数：a--待打印的实向量视图指针
 * 功能：串口打印一个实向量
 * 返回值：TWD_OK--成功
 *			TWD_ERROR--失败
 *			TWD_EPARAM--参数错误
 */
int tw_vprint_f(vsip_vview_f* a)
{
	if(a==NULL)
	{
		printf("tw_vprint_f: Err, a==NULL!\n\n");
		return TWD_EPARAM;
	}
	else
	{
		;
	}
	
	vsip_length i, n;
	vsip_scalar_f x;
	n = vsip_vgetlength_f(a);
	for(i=0; i<n; i++)
	{
		x = vsip_vget_f(a,i);
		printf("%9.4f\t", x);
		if( (i + 1) % 8 == 0 )
		{
			printf("\n");
		}
	}
	printf("\n");
	
	return TWD_OK;
}



/*
 * 内部接口：tw_cvprint_f
 * 参数：a--待打印的复向量视图指针
 * 功能：串口打印一个复向量
 * 返回值：TWD_OK--成功
 *			TWD_ERROR--失败
 *			TWD_EPARAM--参数错误
 */
int tw_cvprint_f(vsip_cvview_f* a)
{
	if(a==NULL)
	{
		printf("tw_cvprint_f: Err, a==NULL!\n\n");
		return TWD_EPARAM;
	}
	else
	{
		;
	}
	
	vsip_length i, n;
	vsip_cscalar_f x;
	n = vsip_cvgetlength_f(a);
	for(i=0; i<n; i++)
	{
		x = vsip_cvget_f(a,i);
		printf("%+9.4f%+9.4fi\t", x.r, x.i);
		if( (i + 1) % 8 == 0 )
		{
			printf("\n");
		}
	}
	printf("\n");
	
	return TWD_OK;
}

/****************************内部接口定义结束****************************/