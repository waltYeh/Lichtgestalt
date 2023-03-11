#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H
#define PI 3.14159265f
typedef struct _IIRFilter {
	float cutoff_freq;
	float a1;
	float a2;
	float b0;
	float b1;		
	float b2;
	float delay_element_1;
	float delay_element_2;
}IIRFilter;
typedef struct _FIRFilter {
	float cutoff_freq;
	float a[5];
	float d[4];
}FIRFilter;
void IIR_set_cutoff_freq(IIRFilter *filter, float cutoff_freq, float smpl_freq);
float IIR_apply(IIRFilter *filter, float sample);
float IIR_reset(IIRFilter *filter, float sample);
#endif
