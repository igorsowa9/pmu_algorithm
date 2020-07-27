/*
    UL call demonstrated:       	  ulAInScan()

    Purpose:                          Performs a continuous scan of the range
                                      of A/D input channels

    Demonstration:                    Displays the analog input data for the
                                      range of user-specified channels using
                                      the first supported range and input mode

    Steps:
    1. Call ulGetDaqDeviceInventory() to get the list of available DAQ devices
    2. Call ulCreateDaqDevice() to to get a handle for the first DAQ device
    3. Verify the DAQ device has an analog input subsystem
    4. Verify the analog input subsystem has a hardware pacer
    5. Call ulConnectDaqDevice() to establish a UL connection to the DAQ device
    6. Get the first supported analog range
    7. Call ulAInScan() to start the scan of A/D input channels
    8. Call ulAInScanStatus() to check the status of the background operation
    9. Display the data for each channel
    10. Call ulAInScanStop() to stop the background operation
    11. Call ulDisconnectDaqDevice() and ulReleaseDaqDevice() before exiting the process.
*/

#include <stdio.h>
#include <stdlib.h>
#include <complex.h>
#include <math.h>
#include "uldaq.h"
#include "utility.h"
#include <time.h>
#include <sys/timeb.h>
#include <sys/time.h>

#define MAX_DEV_COUNT  100
#define MAX_STR_LENGTH 64
#define MAX_SCAN_OPTIONS_LENGTH 256

#define M_PI 3.14159265358979323846
//#define M_E 2.71828182845904523536

int main(void)
{
	int descriptorIndex = 0;
	DaqDeviceDescriptor devDescriptors[MAX_DEV_COUNT];
	DaqDeviceInterface interfaceType = ANY_IFC;
	DaqDeviceHandle daqDeviceHandle = 0;
	unsigned int numDevs = MAX_DEV_COUNT;

	// set some variables that are used to acquire data
	int lowChan = 0;
	int highChan = 0;
	AiInputMode inputMode;
	Range range;
	int samplesPerChannel = 64; // 100 would be desired, but for Fs=5kHz daq feeds in 64-samples batches - if it is not 2^x then works bad somehow!!! 
	// cont.: 32 seems minimum for desired frequencies (test with printing totalcount only) e.g. for 10kHz, 32 i.e. batch every 3.2ms ~ 300 frames/second with 10kHz
	double rate = 6400;
	ScanOption scanOptions = (ScanOption) (SO_DEFAULTIO | SO_CONTINUOUS);
	AInScanFlag flags = AINSCAN_FF_DEFAULT;

	int hasAI = 0;
	int hasPacer = 0;
	int numberOfChannels = 0;
	int index = 0;

	char inputModeStr[MAX_STR_LENGTH];
	char rangeStr[MAX_STR_LENGTH];
	char scanOptionsStr[MAX_SCAN_OPTIONS_LENGTH];

	int chanCount = 0;
	double* buffer = NULL;
	UlError err = ERR_NO_ERROR;

	int i = 0;
	int __attribute__((unused)) ret;
	char c;

	// Get descriptors for all of the available DAQ devices
	err = ulGetDaqDeviceInventory(interfaceType, devDescriptors, &numDevs);

	if (err != ERR_NO_ERROR)
		goto end;

	// verify at least one DAQ device is detected
	if (numDevs == 0)
	{
		printf("No DAQ device is detected\n");
		goto end;
	}

	printf("Found %d DAQ device(s)\n", numDevs);
	for (i = 0; i < (int) numDevs; i++)
		printf("  [%d] %s: (%s)\n", i, devDescriptors[i].productName, devDescriptors[i].uniqueId);

	if(numDevs > 1)
		descriptorIndex = selectDAQDevice(numDevs);

	// get a handle to the DAQ device associated with the first descriptor
	daqDeviceHandle = ulCreateDaqDevice(devDescriptors[descriptorIndex]);

	if (daqDeviceHandle == 0)
	{
		printf ("\nUnable to create a handle to the specified DAQ device\n");
		goto end;
	}

	// verify the specified device supports analog input
	err = getDevInfoHasAi(daqDeviceHandle, &hasAI);
	if (!hasAI)
	{
		printf("\nThe specified DAQ device does not support analog input\n");
		goto end;
	}

	// verify the specified device supports hardware pacing for analog input
	err = getAiInfoHasPacer(daqDeviceHandle, &hasPacer);
	if (!hasPacer)
	{
		printf("\nThe specified DAQ device does not support hardware paced analog input\n");
		goto end;
	}

	printf("\nConnecting to device %s - please wait ...\n", devDescriptors[descriptorIndex].devString);

	// establish a connection to the DAQ device
	err = ulConnectDaqDevice(daqDeviceHandle);

	if (err != ERR_NO_ERROR)
		goto end;

	// get the first supported analog input mode
	err = getAiInfoFirstSupportedInputMode(daqDeviceHandle, &numberOfChannels, &inputMode, inputModeStr);

	if (highChan >= numberOfChannels)
		highChan = numberOfChannels - 1;

	chanCount = highChan - lowChan + 1;

	// allocate a buffer to receive the data
	buffer = (double*) malloc(chanCount * samplesPerChannel * sizeof(double));

	if(buffer == NULL)
	{
		printf("\nOut of memory, unable to create scan buffer\n");
		goto end;
	}

	// get the first supported analog input range
	err = getAiInfoFirstSupportedRange(daqDeviceHandle, inputMode, &range, rangeStr);

	ConvertScanOptionsToString(scanOptions, scanOptionsStr);

	printf("\n%s ready\n", devDescriptors[descriptorIndex].devString);
	printf("    Function demonstrated: ulAInscan()\n");
	printf("    Channels: %d - %d\n", lowChan, highChan);
	printf("    Input mode: %s\n", inputModeStr);
	printf("    Range: %s\n", rangeStr);
	printf("    Samples per channel: %d\n", samplesPerChannel);
	printf("    Rate: %f\n", rate);
	printf("    Scan options: %s\n", scanOptionsStr);
	printf("\nHit ENTER to continue\n");

	ret = scanf("%c", &c);

	// clear the display
	ret = system("clear");

	/// ############################
        // prepare for phasor estimation
	int debug_sliding = 0;

	int f_nom = 50;
        int mem_totalcount = 0;
        int M = 300, ki, k, mi, m, xi, cursor;

	int samples_in_period = rate/f_nom; // 5k/50=100
	int periods_for_calc = 3;
	int sliding_in_samples = 64;
	int phasor_counter = 0;

	int x_buffer_size = (periods_for_calc)*samples_in_period + sliding_in_samples; // (3)*100+64=64
	int daq_sampling_size = samplesPerChannel;

	double* x_buffer = malloc(x_buffer_size*sizeof(double)); // array[400]
	//double x_buffer[400];
	for(xi=0;xi<x_buffer_size;xi++) { // just some initial non-zero values in memory
	        x_buffer[xi] = xi+0.2;
        }

        double complex Wm1[5][300], Wm2[5][300];
        printf("%f + i*%f\n", creal(cexp(-I)), cimag(cexp(-I)));

        for (ki=0; ki<5; ki++){  // for k 1 to 5 i.e. 3+/-2, not universal, depending on M etc.
                k = ki+1;
                for(m=0; m<M; m++){
                        Wm1[ki][m] = cexp(-2*I*M_PI*k*m/M);
                        Wm2[ki][m] = cexp(2*I*M_PI*k*m/M);
                }
        }

	if(debug_sliding==1){
        for(ki=0; ki<5; ki++) {
                for(mi=0;mi<M;mi++) {
                        printf("Wm1[%d,%d]: %f + i*%f\n",ki,mi, creal(Wm1[ki][mi]), cimag(Wm1[ki][mi]));
                }
                printf("\n");
        }
	for(ki=0; ki<5; ki++) {
                for(mi=0;mi<M;mi++) {
                        printf("Wm2[%d,%d]: %f + i*%f\n",ki,mi, creal(Wm2[ki][mi]), cimag(Wm2[ki][mi]));
                }
                printf("\n");
        }}

	// start the acquisition
	err = ulAInScan(daqDeviceHandle, lowChan, highChan, inputMode, range, samplesPerChannel, &rate, scanOptions, flags, buffer);

	if(err == ERR_NO_ERROR)
	{
		ScanStatus status;
		TransferStatus transferStatus;

		// get the initial status of the acquisition
		ulAInScanStatus(daqDeviceHandle, &status, &transferStatus);

		while(status == SS_RUNNING && err == ERR_NO_ERROR && !enter_press() )
		{
			// get the current status of the acquisition
			err = ulAInScanStatus(daqDeviceHandle, &status, &transferStatus);

			// reset the cursor to the top of the display and
			// show the termination message
			//resetCursor();
			//printf("Hit 'Enter' to terminate the process\n\n");
			//printf("Active DAQ device: %s (%s)\n\n", devDescriptors[descriptorIndex].productName, devDescriptors[descriptorIndex].uniqueId);
			//printf("actual scan rate = %f\n\n", rate);

			index = transferStatus.currentIndex;
			//printf("\n\ncurrentScanCount =  %-10llu", transferStatus.currentScanCount);
			printf("\ncurrentTotalCount = %-10llu", transferStatus.currentTotalCount);
			//printf("\ncurrentIndex =      %-10d \n", index);
			//continue;
			if(index >= 0)
			{
				// display the data
				if(1==0){
				for (i = 0; i < chanCount; i++)
				{
                                        //printf("nowy sample%+-10.6f\n", buffer[index]);
                                        //printf("b0: %+-10.6f\n", buffer[0]);
					//printf("b1: %+-10.6f\n", buffer[1]);
                                        //printf("b2: %+-10.6f\n", buffer[2]);
                                        //printf("b3: %+-10.6f\n", buffer[3]);
                                        //printf("b4: %+-10.6f\n", buffer[4]);
					//printf("%+-10.6f\n", buffer[index+3]);
                                        //printf("size buffer: %d\n", (int)(sizeof(buffer))); // it's passed i.e. it's not the real size of the buffer
                                        //printf("size buffer element: %d\n", (int)(sizeof(buffer[0])));
				        //printf("size: %d\n", (int)(sizeof(buffer) / sizeof(buffer[0])));
                                }}
				if(transferStatus.currentTotalCount != mem_totalcount && transferStatus.currentTotalCount % samplesPerChannel == 0){
					mem_totalcount = transferStatus.currentTotalCount; // in order to avoid repeating for the same counter

					if(debug_sliding==1){
					printf("\nCurrent daq buffer:\n");
					for(xi=0;xi<daq_sampling_size;xi++) {
						printf("%+-10.6f ", buffer[xi]);
						if((xi+1)%10==0){printf("\n");}
					}}

					// switch to make space for new ones
					for(xi=0;xi<x_buffer_size-daq_sampling_size;xi++) {
                                                x_buffer[xi] = x_buffer[xi+daq_sampling_size];
                                        }
					// copy new ones to the end
					memcpy(x_buffer+(x_buffer_size-daq_sampling_size), buffer, daq_sampling_size*sizeof(double));

					// current buffer for calculations
					/*printf("\nx buffer: %f samples or (3+1)*sampling/50:\n", (periods_for_calc)*rate/f_nom+sliding_in_samples);
					for(xi=0;xi<x_buffer_size;xi++) {
						printf("%+-10.6f ", x_buffer[xi]);
						if((xi+1)%10==0){printf("\n");}
					}*/

                                        double Xk_row_sum_real, Xk_row_sum_imag;
                                        double complex Xk[5];
                                        double complex Xk_H1, Xk_H2, Xk_H3; // hann in dft domain
					struct timeval timer_usec;
                                        long long int timestamp_usec; /* timestamp in microsecond */
					long long int timestamp_usec_prev = 0;

					if(transferStatus.currentTotalCount > rate){ exit(0); }
					// tringgering phasor estimations or waiting //
					///////////////////////////////////////////////

					if(transferStatus.currentTotalCount < periods_for_calc*samples_in_period) { // 3*rate/50
						printf("\n\nWaiting. Total count so far: %-10llu", transferStatus.currentTotalCount);
					}
					else if((transferStatus.currentTotalCount >= periods_for_calc*samples_in_period) && (transferStatus.currentTotalCount < periods_for_calc*samples_in_period+sliding_in_samples)) {
						// minimum amount for first DFT
						printf("\n\nFirst DFT. Total count so far: %-10llu", transferStatus.currentTotalCount);

						if(debug_sliding==1){
						printf("\nNewest %.0f samples: (3*sampling/50). Oldest excess is cut off.\n", periods_for_calc*rate/f_nom);
						for(xi = sliding_in_samples; xi < x_buffer_size; xi++) {
	                                                printf("%+-10.6f ", x_buffer[xi]);
        	                                        if((xi+1)%10==0){printf("\n");}
                	                        }}
					        for (ki=0; ki<5; ki++){
							Xk_row_sum_real = 0;
							Xk_row_sum_imag = 0;

               						for(mi=0; mi<M; mi++){
           							Xk_row_sum_real = Xk_row_sum_real + x_buffer[sliding_in_samples + mi]*creal(Wm1[ki][mi]); //sliding_in_samples  in order to remove excess from buffer (which is for sliding later)
								Xk_row_sum_imag = Xk_row_sum_imag + x_buffer[sliding_in_samples + mi]*cimag(Wm1[ki][mi]);
               						}
							Xk[ki] = Xk_row_sum_real + Xk_row_sum_imag * I;
							//printf("\nXk%d = %.1f% + .1fi", ki+1, creal(Xk[ki]), cimag(Xk[ki])); // prints each Xk of interest
      						}
						phasor_counter++;
						printf("\n End of Xk calculations for all k of interest. Xk3=%f %fj\nPhasor counter=%d\n", creal(Xk[2]), cimag(Xk[2]), phasor_counter);
						/* Example of timestamp in microsecond. */
                                                if (!gettimeofday(&timer_usec, NULL)) {
                                                timestamp_usec = ((long long int) timer_usec.tv_sec) * 1000000ll + (long long int) timer_usec.tv_usec;
                                                }
                                                else {
                                                timestamp_usec = -1;
                                                }
                                                printf("\n%lld microseconds since epoch. Difference to last: %d\n", timestamp_usec, timestamp_usec-timestamp_usec_prev);
						timestamp_usec_prev = timestamp_usec;
						continue;



						// here it is sufficient for sliding, rest is windowing and interpolation
						Xk_H1 = -0.25*creal(Xk[0]) + 0.5*creal(Xk[1]) - 0.25*creal(Xk[2]) + (-0.25*cimag(Xk[0]) + 0.5*cimag(Xk[1]) - 0.25*cimag(Xk[2]))*I;
						Xk_H2 = -0.25*creal(Xk[1]) + 0.5*creal(Xk[2]) - 0.25*creal(Xk[3]) + (-0.25*cimag(Xk[1]) + 0.5*cimag(Xk[2]) - 0.25*cimag(Xk[3]))*I;
						Xk_H3 = -0.25*creal(Xk[2]) + 0.5*creal(Xk[3]) - 0.25*creal(Xk[4]) + (-0.25*cimag(Xk[2]) + 0.5*cimag(Xk[3]) - 0.25*cimag(Xk[4]))*I;

						/*printf("\nXk_H1=%.1f% + .1fi", creal(Xk_H1), cimag(Xk_H1));
						printf("\nXk_H2=%.1f% + .1fi", creal(Xk_H2), cimag(Xk_H2));
						printf("\nXk_H3=%.1f% + .1fi", creal(Xk_H3), cimag(Xk_H3));
						printf("\n");*/

						double B, alpha, delta_bin, time_win, delta_f, f_estim, A_estim, ph_estim;
						int ki_max, k_max, esign, ki_max_esign;
						double Xk_Habs[3], Xk_Hmax;
						B = 149.5;
						time_win = (float)periods_for_calc / (float)f_nom;

						Xk_Habs[0] = cabsf(Xk_H1)/B;
						Xk_Habs[1] = cabsf(Xk_H2)/B;
						Xk_Habs[2] = cabsf(Xk_H3)/B;

						//printf("\nXk_Habs[0]  = %f", Xk_Habs[0]);
						//printf("\nXk_Habs[1]  = %f", Xk_Habs[1]);
						//printf("\nXk_Habs[2]  = %f", Xk_Habs[2]);

						Xk_Hmax = Xk_Habs[1];
						ki_max = 1; // assumed for now
						k_max = ki_max + 2;
						if (Xk_Habs[0]>Xk_Habs[2]) {
							esign = -1;
						} else {
							esign = 1;
						}
						//printf("\nesign = %d", esign);
						ki_max_esign = ki_max + esign;
						//printf("\nki_max_esign = %d", ki_max_esign);
						alpha = Xk_Habs[ki_max] / Xk_Habs[ki_max_esign];
						delta_bin = esign * (2-alpha) / (1+alpha);
						delta_f = 1/time_win;
						//printf("\n %f %f %f %f", alpha, delta_bin, time_win, delta_f);

						f_estim = (k_max + delta_bin) * delta_f;
						A_estim = 2*Xk_Hmax*(M_PI*delta_bin*(1-delta_bin*delta_bin))/sin(M_PI*delta_bin);
						ph_estim = cargf(Xk_H2)-M_PI*delta_bin;

						printf("\n\nf_estim = %fHz \tA_estim = %fV \tph_estim = %f deg.\n", f_estim, A_estim, ph_estim*180/M_PI);
						//fprintf(stdout, "%lu\n", (unsigned long)time(NULL));

						/* Example of timestamp in millisecond. */
						/*struct timeb timer_msec;
						long long int timestamp_msec;
  						if (!ftime(&timer_msec)) {
    							timestamp_msec = ((long long int) timer_msec.time) * 1000ll + (long long int) timer_msec.millitm;
  						}
  						else {
    							timestamp_msec = -1;
  						}
  						printf("%lld milliseconds since epoch\n", timestamp_msec); */

						/* Example of timestamp in microsecond. */
						struct timeval timer_usec;
  						long long int timestamp_usec; /* timestamp in microsecond */
  						if (!gettimeofday(&timer_usec, NULL)) {
    						timestamp_usec = ((long long int) timer_usec.tv_sec) * 1000000ll + (long long int) timer_usec.tv_usec;
  						}
  						else {
    						timestamp_usec = -1;
  						}
  						printf("%lld microseconds since epoch\n", timestamp_usec);

						//exit(0);
			                        //printf("Wm1[%d,%d]: %f + i*%f\n",ki,mi, creal(Wm1[ki][mi]), cimag(Wm1[ki][mi]));

					}
					else if(transferStatus.currentTotalCount >= periods_for_calc*samples_in_period+sliding_in_samples) {
						// sufficient amount for sliding i.e. minimum + one period
						printf("\n\nSliding possible. Total count so far: %-10llu", transferStatus.currentTotalCount);
						printf("\nXk before sliding:");
						//for (ki=0;ki<5;ki++) {
						//	printf("\nXk%d = %.1f% + .1fi", ki+1, creal(Xk[ki]), cimag(Xk[ki]));
						//}
						printf("\nXk3: abs:%f  deg:%f", cabs(Xk[2]), carg(Xk[2])*180/M_PI);

						//double complex Xk_slide[5]; // for storing new Xk for 5 k of interest
						double complex into0, slide; // for storing temporary stuff during slides
						int n, sui, n_m;
						double re_into0, im_into0;
						for(ki=0;ki<5;ki++) {
							k=ki+1;
							into0 = Xk[ki];
							//printf("\ninto0: %f 1i*%f", creal(into0), cimag(into0));
							slide = 0;
							for(sui=0; sui<sliding_in_samples; sui++){
								n = M+sui;
								n_m = n%M;

								re_into0 = creal(into0) + creal(Wm1[ki][n_m]) * (-x_buffer[n-M] + x_buffer[n]);
								im_into0 = cimag(into0) + cimag(Wm1[ki][n_m]) * (-x_buffer[n-M] + x_buffer[n]);

								into0 = re_into0 + im_into0*I;
								slide = Wm2[ki][n_m+1] * into0;

								if(debug_sliding==1)
								{
									printf("\nki=%d n=%d n_m=%d Wm1[%d,%d]= %f + i*%f",ki,n,n_m,ki,n_m, creal(Wm1[ki][n_m]), cimag(Wm1[ki][n_m]));
									printf("\ninto0:%f %fi\tWm1:%f %fi\t samples sum:%f", creal(into0), cimag(into0), creal(Wm1[ki][n_m]), cimag(Wm1[ki][n_m]), (-x_buffer[n-M] + x_buffer[n]) );
									printf("\n%d %d remove(%d): %f add(%d):%f ... into0_abs:%f  into0_deg:%f  slide_abs:%f  slide_deg:%f",n_m, n, n-M, x_buffer[n-M], n, x_buffer[n],
										cabs(into0),
										carg(into0)*180/M_PI,
										cabs(slide),
                                                                        	carg(slide)*180/M_PI);
								}
							}
							Xk[ki] = slide;
						}
						if(debug_sliding==1)
						{
							printf("\nXk_slide: after sliding:");
                                                	for (ki=0;ki<5;ki++) {
                                                        	printf("\nXk%d = %.1f% + .1fi", ki+1, creal(Xk[ki]), cimag(Xk[ki]));
                                                	}
						}
						phasor_counter++;
						printf("\nXk3 after slide: abs:%f  deg:%f", pow(pow(creal(Xk[2]),2)+pow(cimag(Xk[2]),2),0.5), carg(Xk[2])*180/M_PI);
						printf("\nPhasor counter=%d", phasor_counter);
						/* Example of timestamp in microsecond. */
                                                if (!gettimeofday(&timer_usec, NULL)) {
                                                timestamp_usec = ((long long int) timer_usec.tv_sec) * 1000000ll + (long long int) timer_usec.tv_usec;
                                                }
                                                else {
                                                timestamp_usec = -1;
                                                }
                                                printf("\n%lld microseconds since epoch. Difference to last: %f\n", timestamp_usec, timestamp_usec_prev);
						timestamp_usec_prev = timestamp_usec;
						continue;

						//printf("\n\ntotal in modulo: %-10llu \n", transferStatus.currentTotalCount);
                	                        printf("\nbuffer elements: \n");
        	                                for (i=0; i<samplesPerChannel; i++)
						{
                                        		printf("%+-10.6f",buffer[i]);
							if((i+1)%10==0){printf("\n");}
                                		}
					}
				}

				usleep(1000);
			}
		}

		// stop the acquisition if it is still running
		if (status == SS_RUNNING && err == ERR_NO_ERROR)
		{
			err = ulAInScanStop(daqDeviceHandle);
		}
	}

	// disconnect from the DAQ device
	ulDisconnectDaqDevice(daqDeviceHandle);

end:

	// release the handle to the DAQ device
	if(daqDeviceHandle)
		ulReleaseDaqDevice(daqDeviceHandle);

	// release the scan buffer
	if(buffer)
		free(buffer);

	if(err != ERR_NO_ERROR)
	{
		char errMsg[ERR_MSG_LEN];
		ulGetErrMsg(err, errMsg);
		printf("Error Code: %d \n", err);
		printf("Error Message: %s \n", errMsg);
	}

	return 0;
}
