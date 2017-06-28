/*
Copyright (c) 2017, BigCat Wireless Pvt Ltd
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.



THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**
* @defgroup DPDApplication
* @brief Application for DPD adaptation algorithm.
*/


/*******************************************************************************
* Include public/global header files
*******************************************************************************/
#include<stdio.h>
#include<math.h>
#include<stdlib.h>
#include <stdbool.h>


/*******************************************************************************
* Include private header files
*******************************************************************************/
#include "dpd_datatype.h"
#include "dpd_input_parameters.h"
#include "datatype.h"
#include "dpd_firmware.h"


Hepta_comp *CoRx,*G,*GxG,*GxCTx,*CTx,*final_out;
Hepta64B matrix[(2*N)][(4*N)];

int DebugLevel=DEBUG_LEVEL_ERROR;


/**
* @brief Computes the condition number
* @param buffer G'xG matrix
* @param inv_buffer inv(G'xG) matrix
* @return Condition number of the G'xG matrix
*/

Hepta64B hepta_dpd_condition_num(Hepta_comp *buffer,Hepta64B inv_buffer[(2*N)][(4*N)])
{
	Hepta32I i, j;
	Hepta64B norm1A,norm1Ainv,sum_array[N], temp_abs = 0;
	for (j = 0; j < N; j++)
	{
		for (i = 0; i < N; i++)
		{
			temp_abs += sqrt(((buffer+i+j)->r*(buffer+i+j)->r) + ((buffer+i+j)->im*(buffer+i+j)->im));
		}
		sum_array[j] = temp_abs;
		temp_abs = 0;
	}
	norm1A = sum_array[0];
	for (i = 1; i < N; i++)
	{
		if (sum_array[i]>norm1A)
			norm1A = sum_array[i];
	}
	
	//now calculate norm-1 for inverse matrix
	for (i = 0; i < N; i++)
	{
		for (j=(2*N); j<(3*N); j++)
		{
			temp_abs += sqrt((inv_buffer[i][j] * inv_buffer[i][j]) + (inv_buffer[i + N][j] * inv_buffer[i + N][j]));
		}
		sum_array[i] = temp_abs;
		temp_abs = 0;
	}
	norm1Ainv = sum_array[0];
	for (i = 1; i < N; i++)
	{
		if (sum_array[i]>norm1Ainv)
			norm1Ainv = sum_array[i];
	}
	
	return (norm1A*norm1Ainv);
}


/**
* @brief Computes the mean power of the capture data
* @param buffer Reverse path capture buffer data
* @return Mean power in (dB) of the capture data
*/


Hepta32F hepta_dpd_mean_power(Hepta_comp * buffer,hepta_u32 length)
{
	Hepta32I i;
	Hepta32F temp_abs = 0;
	for (i = 0; i < length; i++)
	{
		temp_abs += (((buffer + i)->r*(buffer + i)->r) + ((buffer + i)->im*(buffer + i)->im));
	}
	temp_abs /= length;
	temp_abs = sqrtf(temp_abs);
	temp_abs = 20 * log10(temp_abs);
	printf("Corx Mean power = %f\n",temp_abs);
	return temp_abs;
}


/**
* @brief Compute and checks if the capture has more peaks
* @param buffer Reverse path capture buffer
* @param peak_threshold Maximum peak
* @param max_peak_count Number of peaks greater than @param peak_threshold allowed
* @retval true Capture is not peaky
* @retval false Capture is peaky
*/

bool hepta_dpd_peaky_capture(Hepta_comp *buffer,hepta_u32 length, Hepta32F peak_threshold, Hepta32I max_peak_count)
{
	printf("peak threshold = %f\n",peak_threshold);
	Hepta32I i,count=0;
	Hepta32F temp_abs;
	for (i = 0; i < length; i++)
	{	temp_abs = sqrtf( ( (buffer+i)->r*(buffer+i)->r) + ( (buffer+i)->im*(buffer+i)->im) );
		if (temp_abs>peak_threshold)
			count++;
	}

	printf("Peaky capture number of peaks greater than %f = %d\n",peak_threshold, count);

	if (count > max_peak_count)
		return false;
	else
		return true;
}


/**
* @ingroup DPD adaptation algorithm
* @brief Initialize global pointers.
*/

void init_dpd_adaptation()									
{	
	CoRx =(Hepta_comp*)malloc(samplesize*sizeof(Hepta_comp));
	CTx  =(Hepta_comp*)malloc(CTxlength*sizeof(Hepta_comp));
    G=(Hepta_comp*)malloc(Glength*sizeof(Hepta_comp));
	GxG=(Hepta_comp*)malloc(GxGlength*sizeof(Hepta_comp));
	GxCTx=(Hepta_comp*)malloc(GxCTxlength*sizeof(Hepta_comp));
	final_out=(Hepta_comp*)malloc(N*sizeof(Hepta_comp));
}

/**
* @ingroup DPD adaptation algorithm
* @brief Construct G matrix from CoRx vector.
*/

void CovarMatrix()								
{	Hepta32I i,j,k,l;
	j=corxlength;k=2*corxlength;l=3*corxlength;
	Hepta64B temp_abs;
	for(i=0;i<7;i++)
	{	(G+i)->r=0;(G+i)->im=0;
		(G+j)->r=0;(G+j)->im=0;
		(G+k)->r=0;(G+k)->im=0;
		(G+l)->r=0;(G+l)->im=0;
		j++;k++;l++;
	}
	for(i=7;i<corxlength;i++)
	{	
		(G+i)->r=(CoRx+(i-7))->r;
		(G+i)->im=(CoRx+(i-7))->im;
		temp_abs=sqrt(((G+i)->r*(G+i)->r)+((G+i)->im*(G+i)->im));
		(G+j)->r=(G+i)->r*temp_abs*temp_abs;
		(G+j)->im=(G+i)->im*temp_abs*temp_abs;

		(G+k)->r=(G+j)->r*temp_abs*temp_abs;
		(G+k)->im=(G+j)->im*temp_abs*temp_abs;

		(G+l)->r=(G+k)->r*temp_abs*temp_abs;
		(G+l)->im=(G+k)->im*temp_abs*temp_abs;

		j++;k++;l++;

	}
}

/**
* @ingroup DPD adaptation algorithm
* @brief Perform Matrix multiplication G' x G
*/

void MM1_Gt_x_G()							
{	Hepta32I h,i,j,k,l,m,n=0;
	Hepta64B tempr=0,tempi=0;

	for(h=0;h<MD;h++)//h controls rows of Gt matrix
	{	for(i=0;i<4;i++)//i controls the offset of Gt matrix
		{
		j=MD-h-1;
			for(k=0;k<MD;k++)//k controls for one full row of output
			{	for(l=0;l<4;l++)//l controls the offset of one row
				{	for (m=(MD-k-1);m<(corxlength-k);m++)
					{	if ((h)&&(k<h))             
						{tempr=0;tempi=0;}
						else if ((i)&&(k==0)&&(l<i))
						{tempr=0;tempi=0;}
						else
						{tempr +=( ( (G+j+(i*corxlength))->r*(G+m+(l*corxlength))->r) + ( (G+j+(i*corxlength))->im*(G+m+(l*corxlength))->im)  );
						 tempi +=( ( (G+j+(i*corxlength))->r*(G+m+(l*corxlength))->im) - ( (G+j+(i*corxlength))->im*(G+m+(l*corxlength))->r)  ); 
						j++;}
					}
					(GxG+n)->r=tempr;
					(GxG+n)->im=tempi;
					n++;tempr=0;tempi=0;j=(MD-h-1);
				}
			}
		}
	}
	k=N-1;
	for (i=1;i<GxGlength;i=i+N+1)
	{	k--;
		for (j=i,l=1;j<=(i+k);j++,l++)
		{	(GxG+(i-1)+(l*N))->r=(GxG+j)->r;
		(GxG + (i - 1) + (l*N))->im = (-1 * (GxG + j)->im);/**< complex symmetric property.*/
		}
	}
}

/**
* @ingroup DPD adaptation algorithm
* @brief Perform Matrix multiplication G' x CTx
*/

void MM2_Gt_x_CTx()								
{	Hepta32I h,i,j,l=0,n=0;
	Hepta64B tempr=0,tempi=0;
	for(h=0;h<MD;h++)
	{	for(i=0;i<4;i++)
		{	for(j=(MD-h-1);j<(corxlength-h);j++)
			{			
				tempr +=( ((G+j+(i*corxlength))->r*(CTx+l)->r) + ((G+j+(i*corxlength))->im*(CTx+l)->im)  );
				tempi +=( ((G+j+(i*corxlength))->r*(CTx+l)->im) - ((G+j+(i*corxlength))->im*(CTx+l)->r)  );
				l++;			
			}
			(GxCTx+n)->r=tempr;
			(GxCTx+n)->im=tempi;
			n++;tempr=0;tempi=0;l=0;
		}
	}
}

/**
* @ingroup DPD adaptation algorithm
* @brief Perform Matrix Inversion of G' x G matrix using Gauss Jordan elimination 
* @retval 0 Matrix inversion completed successfully
* @retval -1 Matrix cannot be inverted
*/

hepta_32 MI_inv()
{   Hepta32I n=N,cn=2*N; 
	Hepta32I i,j,k;
    Hepta32I	p=0;
	Hepta64B temp,a,ratio;
    for(i = 0; i < n; i++)
	{
        for(j = 0; j < n; j++)
		{
            matrix[i+n][j+n] = matrix[i][j] = (GxG+p)->r;
			matrix[i+n][j] =  (GxG+p)->im;
			matrix[i][j+n]= (-1)*matrix[i+n][j];
			p++;
        }
    }

	for(i = 0; i < cn; i++)/**< Append identity matrix .*/
	{		
        for(j = cn; j < 2*cn; j++)
		{
            if(i==(j-cn))
                matrix[i][j] = 1.0;
            else
                matrix[i][j] = 0.0;
        }
    }

    for(i = 0; i < cn; i++)/**< Gauss Jordan elimination process.*/
	{
        for(j = 0; j < cn; j++)
		{
            if(i!=j)
			{	
				if(matrix[i][i]!=0)
				{
					ratio = matrix[j][i]/matrix[i][i];
					for(k = 0; k < 2*cn; k++)
						matrix[j][k] -= ratio * matrix[i][k];
				}
				else
				{
					for(k=i+1 ; k<cn ; k++)
					{	if(matrix[k][i]!=0)
						{	
							for(p=0;p<2*cn;p++)
							{	
								temp=matrix[k][p];
								matrix[k][p]=matrix[i][p];
								matrix[i][p]=temp;
							}
						}
						else if(k==(cn-1))
						{
							printf("\nTHE MATRIX CANNOT BE INVERTED");
							return -1;
						}
					}
				}
            }
        }
    }

    for(i = 0; i < cn; i++)				/**< Normalize the diagnols.*/
	{    a = matrix[i][i];
        for(j = 0; j < 2*cn; j++)
		{    matrix[i][j] /= a;
        }
    }
    return 0;
}

/**
* @ingroup DPD adaptation algorithm
* @brief Perform Matrix Multiplication (inv(G'xG)) x (G' x CTx) 
*/

void MI_x_MM2()										
{	Hepta32I i,j,k,l=0;
	Hepta64B tempr=0,tempi=0;
	for (i=0;i<N;i++)
	{	
		for (j=(2*N),k=0 ; j<(3*N) && k<N ; j++,k++)
		{		tempr +=( (matrix[i][j]*(GxCTx+k)->r) - (matrix[i+N][j]*(GxCTx+k)->im)  );
				tempi +=( (matrix[i+N][j]*(GxCTx+k)->r) + (matrix[i][j]*(GxCTx+k)->im)  );
		}
		(final_out+l)->r=tempr;
		(final_out+l)->im=tempi;
		l++;tempr=0;tempi=0;
	}
}

/**
* @ingroup DPD adaptation algorithm
* @brief Write to file for verifying functionality at each step. 
*/

void fileoperation()								
{	Hepta32I i,j;
	FILE *fp;
	fp=fopen("DPD_NEON_NIMA.txt","w+");
	if(fp==NULL)
		printf("\nCOULDN'T OPEN THE FILE");
	else
	{	for(i=0;i<corxlength;i++)					//send the input CORX
		fprintf(fp,"%0.20lf %0.20lf\n",(G+i)->r,(G+i)->im);
		fprintf(fp,"\n\n");						//to verify the Gt_x_G
		for(i=0;i<GxGlength;i++)
		fprintf(fp,"%0.20lf %0.20lf\n",(GxG+i)->r,(GxG+i)->im);
		fprintf(fp,"\n\n");						//to verify the Gt_x_CTx
		for(i=0;i<GxCTxlength;i++)
		fprintf(fp,"%0.20lf %0.20lf\n",(GxCTx+i)->r,(GxCTx+i)->im);
		fprintf(fp,"\n\n");							//send the input CTx
		for(i=0;i<(CTxlength);i++)	
		fprintf(fp,"%0.20lf %0.20lf\n",(CTx+i)->r,(CTx+i)->im);
		fprintf(fp,"\n\n");
		for(i=0;i<N;i++)							//to verify the output of inverse function
		{	for(j=(2*N);j<(3*N);j++)
			{	fprintf(fp,"%0.20lf %0.20lf\n",matrix[i][j],matrix[i+N][j]);
			}
		}
		fprintf(fp,"\n\n");							//to verify the final output
		for(i=0;i<N;i++)
		fprintf(fp,"%0.20lf %0.20lf\n",(final_out+i)->r,(final_out+i)->im);
	}
	fclose(fp);
}

/**
* @ingroup DPD adaptation algorithm
* @brief Split the input 32bit unsigned vector from hardware into 16 bits real and imaginary vectors to form CoRx and CTx 
* @param ptr_iq 32bits input vector from hardware
* @param sym Store the 16 bits real and imaginary values after performing splitting
*/

Hepta64B split_32UBuf (pHepta32U ptr_iq,Hepta_comp *sym,unsigned int size)
{       Hepta32I i;
        pHepta16U ptr ;
        Hepta64B scale ;
        Hepta64B max_abs = -1;
        Hepta64B max_abs_current;


        scale = 32768; //  256*pow(10,0);
        ptr = (pHepta16U) ptr_iq;
        for (i=0;i<size;i++)
        { 	(sym+i)->r = (Hepta64B) ( ( (Hepta16S )*ptr++)  / (1 * scale) );
            (sym+i)->im = (Hepta64B)( ( (Hepta16S )*ptr++)  / (1 * scale) );  //32768=2^15 (written this way to avoid pow function being called again and again)
            max_abs_current = sqrt((((sym+i)->r)*((sym+i)->r))+(((sym+i)->im)*((sym+i)->im)));

            if(max_abs_current > max_abs)
            {
            	max_abs = max_abs_current;
            }
        }
        for (i=size;i<samplesize;i++)
		{ 	(sym+i)->r = 0;
			(sym+i)->im = 0;
		}
        printf("Final Max_abs%f\n",max_abs);

        return max_abs;
}

/**
* @ingroup DPD adaptation algorithm
* @brief Free memory allocated in function init() 
*/

//void kill()
/*{free(G);
free(GxG);
free(GxCTx);
free(CTx);
free(final_out);
}*/

/**
* @ingroup DPD adaptation algorithm
* @brief Call all the relevant functions which make up the DPD adaptation algorithm
* @param pcorx_buf Pointer to 32 bit unsigned CoRx vector from hardware
* @param pctx_buf Pointer to 32 bit unsigned CTx vector from hardware
*/

Hepta_comp* dpd_algorithm(pHepta32U pcorx_buf, pHepta32U pctx_buf)
{	

	split_32UBuf(pctx_buf,CTx,samplesize);					//printf("CTX SPLIT \n");
    split_32UBuf(pcorx_buf,CoRx,samplesize);				//printf("CoRX SPLIT \n");
    CovarMatrix();
	MM1_Gt_x_G();
	MM2_Gt_x_CTx();
	MI_inv();
	MI_x_MM2();
	//fileoperation();					//can be commented because required only for verification purposes
	return (final_out);
}

//STARTING POINT OF THE PROGRAM //CoRx_BUF and CTx_BUF are the unsigned int pointers obtained from hardware
#if 1
Hepta32I main()
{	pHepta32U CoRx_BUF,CTx_BUF;
	CoRx_BUF =(pHepta32U )malloc(corxlength*sizeof(Hepta32U));			//CREATE BUFFER FOR CoRx and CTx where CoRx->output of power amplifier and CTx->input of power amplifer
	CTx_BUF  =(pHepta32U )malloc(CTxlength*sizeof(Hepta32U));			//CoRx and CTx are unsigned 32 bit pointers



	/*FILE *fp;Hepta32I i;								//read input from file for testing purposes
	fp=fopen("inputFromMATLAB.txt","r");
	if(fp==NULL)
		printf("\nCOULDN'T OPEN THE FILE");
	else
	{	for(i=0;i<corxlength;i++)
		{	fscanf(fp,"%f %f %f %f\n",(CoRx_BUF+i),(CTx_BUF+i));
		}
	}
	fclose(fp);*/
	/*Hepta32I i;										//INITIALIZATION OF CTX and CoRx for TESTING purposes
	for (i=0;i<CTxlength;i++)
	{	*(CTx_BUF+i)=  (Hepta32U)  rand()%8 + rand()%8 * 65536;
	}
	for (i=0;i<corxlength;i++)
	{	*(CoRx_BUF+i)= (Hepta32U)  rand()%8 + rand()%8 * 65536;				//(CoRx_BUF+i)=(i+100) + ((i+100) * 32768) ; //  rand()%8;
	}*/



	init_dpd_adaptation();
	//Hepta_comp *result;
	
	//result=dpd_algorithm(CoRx_BUF,CTx_BUF);					//FINAL DPD COEFFICIENTS ARE AVAILABLE IN "result"
	
	//kill();
	free(CoRx_BUF);
	free(CTx_BUF);
getchar();
return 0;
}
#endif
