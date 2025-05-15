#include "math.h"

#ifndef _HLP_H_Included_
#define _HLP_H_Included_
#include "Hlp.h"
#endif

#ifndef _DST_H_Included_
#define _DST_H_Included_
#include "DST.h"
#endif

#ifndef _GENALG_H_Included_
#define _GENALG_H_Included_
#include "GenAlg.h"
#endif

#include <iostream>


/*******************************************************************
* Profile:				To segment the array (1D) by continuinty
* 
* Inputs:
*	pdValIn:			data in	
*	Ele_Num:			element number
*	dDTh:				difference threshold
*	nStep:				search step
*	                    continuinty is calculated between current and seach elements
*	                    0,     only the current and next element will be checked
*	                    1...n, n elements after the next element will be checked
*	Grp_Ele_Min:		minimum element number in each group
*						the group with less elments will be deleted
* 
* Outputs:
*	pnGrpID:			group ID
*
* Revision history:
*	03/2012				version 1.0, created by Xinlian Liang 
*
* Remarks:
*
*******************************************************************/
bool Profile(const double* pdValIn, const long lEleNum, const double dDTh, const int nStep, int* pnGrpID)
{
	// Interation parameters
    long lEle = 0;
    long i_ele = 0, j_ele = 0;

	// Detection parameters
	long lCur = 0, lNext = 0;
    long lBufSta = 0, lBufEnd = 0;
	long lGrpSta = 0, lGrpEnd = 0;
    double dVal_Cur = 0.0f;
    double dDiff_Cur = 0.0f;
    
    long lEleID = 0;    
    long lGrpNum = 0;
	long lGrpIDCur = lGrpNum;
    
    // pnGrpID: Group ID
    for ( lEle = 0; lEle < lEleNum; lEle++ )
	{            
    	pnGrpID[lEle] = INSIGNIFICANCE;
    }
    
    // Group information
    GrpStaic* pGrpInfo = new GrpStaic[lEleNum];
    if ( NULL == pGrpInfo ) {
		return false;
    }
    for( lEle = 0; lEle < lEleNum; lEle++ )
    {
		(pGrpInfo + lEle)->lGrpID = INSIGNIFICANCE;
		(pGrpInfo + lEle)->lStaID = INSIGNIFICANCE;
		(pGrpInfo + lEle)->lEndID = INSIGNIFICANCE;
	}
    
    
    // Detection
	for ( lEle = 0; lEle < lEleNum; lEle++ )
	{	// if "distance is -9999" or "group ID don't equal to -9999"
    	if ( Val_Equ(pdValIn[lEle], (double)INSIGNIFICANCE) || !Val_Equ(pnGrpID[lEle], (int)INSIGNIFICANCE) )
        {
        	continue;
        }
                
        // Find a new start element
		// Group ID begins at 0
		lGrpIDCur = lGrpNum;
        pnGrpID[lEle] = lGrpIDCur;
        
		(pGrpInfo + lGrpIDCur)->lGrpID = lGrpIDCur;
		(pGrpInfo + lGrpIDCur)->lStaID = lEle;
		(pGrpInfo + lGrpIDCur)->lEndID = lEle;

        lNext = lEle;
		lCur = -1;
        
        lGrpNum += 1;	// For next group

        while( lNext > lCur )
        {
            lCur = lNext;

        	// Current element
            dVal_Cur = pdValIn[ lCur ];
            
        	// Buffer space
         	lBufSta = lCur + 1;
            lBufSta = lBufSta < lEleNum ? lBufSta : lEleNum;
            lBufEnd = lBufSta + nStep;
            lBufEnd = lBufEnd < lEleNum ? lBufEnd : lEleNum;
            
        	// Detecting in buffer
            for ( i_ele = lBufSta; i_ele < lBufEnd; i_ele++ )
            {
				// Meaningless element 
                if ( Val_Equ(pdValIn[i_ele], (double)INSIGNIFICANCE) )
                {
                    continue;
                }
				// Element belongs to the same group
            	if ( lGrpIDCur == pnGrpID[i_ele] )
                {
                	if ( lNext == lCur )
                	{
                    	lNext = i_ele;
                	}
                	continue;
             	}
                
				// Element does not belong to the same group
                dDiff_Cur = fabs( pdValIn[ i_ele ] - dVal_Cur );
                    
                if( dDTh > dDiff_Cur )
                {
					// Element does not belong to any group
                	if ( INSIGNIFICANCE == pnGrpID[i_ele] )
                    {
                    	pnGrpID[i_ele] = lGrpIDCur;
                        if ( i_ele > (pGrpInfo + lGrpIDCur)->lEndID )
                        {
                            (pGrpInfo + lGrpIDCur)->lEndID = i_ele;
                        }
						
						// Start element for next iteration
						if ( lNext == lCur )
						{
                    		lNext = i_ele;
						}
                    }
                    else // Element belongs to another group, Gi.
                    {
						// Current group ID
                    	lEleID = pnGrpID[lCur];
						// Modify current group ID to Gi
						lGrpIDCur = pnGrpID[i_ele];
                        
						// Merge current group to Gi
						lGrpSta = (pGrpInfo + lEleID)->lStaID;
                        lGrpEnd = (pGrpInfo + lEleID)->lEndID;
                        for ( j_ele = lGrpSta; j_ele <= lGrpEnd; j_ele++ )
                        {
							if ( lEleID == pnGrpID[j_ele] )
                            {
								pnGrpID[j_ele] = lGrpIDCur;
                            }
                        }
						
						// Modify Gi information
						if ( lGrpSta < (pGrpInfo + lGrpIDCur)->lStaID )
                        {
							(pGrpInfo + lGrpIDCur)->lStaID = lGrpSta;
                        }
						if ( lGrpEnd > (pGrpInfo + lGrpIDCur)->lEndID )
                        {
							(pGrpInfo + lGrpIDCur)->lEndID = lGrpEnd;
                        }

						// Delete current group
						(pGrpInfo + lEleID)->lGrpID = INSIGNIFICANCE;
						(pGrpInfo + lEleID)->lStaID = INSIGNIFICANCE;
						(pGrpInfo + lEleID)->lEndID = INSIGNIFICANCE;


						// Stop searching in current buffer 
						// and start it again with new Group ID, lGrpIDCur
						lNext = lCur;
						lCur  = -1;
						break;
                    } // End of if-else
                } // End of value difference (if)

            } // End of buffer (for)
        } // End of one group (while)
    } // End of profile (for)

	delete pGrpInfo;

	return true;
}


/*******************************************************************
* 
* GrpInfo:		Group statistic information
* 
* Inputs:
* pnGrpID:		Group ID array, n-by-1
*				bigen with 0, no meaning ones marked by INSIGNIFICANCE
* lEleNum:		Input array dimension, n
* lGrpEleNumTh:	Group element number threshold, minimum is 0
*				=0, any meaningful group, whose element is not 0 will be reserved
*				>0, qualified group (larger than) will be reserved
*
* Outputs:
* plGrpNum:		Group number
*				0 if no qualified group
* pGrpEle:		Group statistic information, 
*               Group ID, Element Number, Start&End Position
*				NULL if no qualified group
*
* Revision history:
* 05/2008       version 1.0, created by Xinlian Liang 
*
* Remarks:
*
*******************************************************************/
bool GrpInfo(const int* pnGrpID, const long lEleNum, const long lGrpEleNumTh, long* plGrpNum, GrpStaic** pGrpEle)
{
	long lEle = 0;
    long lGrp = 0;
	long lGrpNum = 0;
	long lGrpNumCnt = 0;
	
	// Maximum group ID, rough group number
	long lMaxID = ArrayMax( pnGrpID, lEleNum );
	lGrpNum = pnGrpID[lMaxID];
	lGrpNum += 1;
    
	// Gruop information: Group ID, Element Number, Start&End Position
    *pGrpEle = new GrpStaic[ lGrpNum ];
    if ( NULL == pGrpEle ) {
		return false;
    }
    for( lEle = 0; lEle < lGrpNum; lEle++ )
    {
		(*pGrpEle + lEle)->lGrpID  = INSIGNIFICANCE;
		(*pGrpEle + lEle)->lEleNum = 0;
		(*pGrpEle + lEle)->lStaID  = INSIGNIFICANCE;
		(*pGrpEle + lEle)->lEndID  = INSIGNIFICANCE;
	}
    
	for( lEle = 0; lEle < lEleNum; lEle++ )
    {
		lGrp = pnGrpID[lEle];

		if ( 0 > lGrp )	{
			continue;
		}
            
        (*pGrpEle + lGrp)->lEleNum += 1;            
        if ( 1 == (*pGrpEle + lGrp)->lEleNum )
        {
			(*pGrpEle + lGrp)->lGrpID = lGrp;
			(*pGrpEle + lGrp)->lStaID = lEle;
            (*pGrpEle + lGrp)->lEndID = lEle;
        }
        else
        {
            (*pGrpEle + lGrp)->lEndID = lEle;
        }
    }

	// Adjustment
	// group, where elements are less than 'lEleNumTh', will be deleted
	// the one, where no elements included, will also be deleted

    // Meaningful group number
	long lEleNumTh = 0;
	if ( 0 < lGrpEleNumTh )
	{
		lEleNumTh = lGrpEleNumTh;
	}

	long lGrpNumTemp = lGrpNum;
    for( lGrp = 0; lGrp < lGrpNum; lGrp++ )
    {
        if( lEleNumTh >= (*pGrpEle + lGrp)->lEleNum )
        {
			(*pGrpEle + lGrp)->lGrpID = INSIGNIFICANCE;
			lGrpNumTemp--;
        }
    }
	// New group information
	if ( 0 == lGrpNumTemp )
	{
		lGrpNum = 0;
		delete[] pGrpEle;
		pGrpEle = NULL;
	}
	else if ( lGrpNumTemp != lGrpNum )
	{
		// original information
		long lGrpNumOrg = lGrpNum;
		GrpStaic* pGrpEleTemp = NULL;
		pGrpEleTemp = *pGrpEle;
		// new information
		lGrpNum = lGrpNumTemp;
		*pGrpEle = new GrpStaic[ lGrpNum ];
		if ( NULL == pGrpEle ) {
			return false;
		}
			
		long lGrpCnt = 0;
		for( lGrp = 0; lGrp < lGrpNumOrg; lGrp++ )
		{
        	if( INSIGNIFICANCE == pGrpEleTemp[lGrp].lGrpID )	{
				continue;
			}
				
			(*pGrpEle + lGrpCnt)->lGrpID  = pGrpEleTemp[lGrp].lGrpID;
			(*pGrpEle + lGrpCnt)->lEleNum = pGrpEleTemp[lGrp].lEleNum;
			(*pGrpEle + lGrpCnt)->lStaID  = pGrpEleTemp[lGrp].lStaID;
			(*pGrpEle + lGrpCnt)->lEndID  = pGrpEleTemp[lGrp].lEndID;
			lGrpCnt++;
		}
		delete[] pGrpEleTemp;
	}
	else
	{
		//if ( lGrpNumTemp == lGrpNum )
	}

	*plGrpNum = lGrpNum;

	return true;
}

/*******************************************************************
* 
* Grp_ID_Ord :		To rearange ID in sequence and delete virtual ID
* 
* Inputs:
* pnGrp_ID:			Group ID array, n-by-1
* lEle_Num:			Input array dimension, n
* nGrp_Ele_Min:		Group element number threshold
*					=0, any group with any element will be reserved, only ID will be rearranged in order
*					>0, qualified group (larger than) ID will be reserved
*					if no qualified group, all group ID is set INSIGNIFICANCE
* 
* Outputs:
*					Input pnGrp_ID will be changed, ID will be in sequence
*
* Revision history:
* 05/2008			version 1.0, created by Xinlian Liang 
* 03/2012			minor update
*
* Remarks:
*
*******************************************************************/
bool Grp_ID_Ord(int* pnGrp_ID, const long lEle_Num, const int nGrp_Ele_Min)
{
	// Group information
	long lGrp = 0, lEle = 0;
	long lGrp_Cnt = 0, lGrp_ID  = 0;
	long lEle_Sta = 0, lEle_End = 0;
	// Group statistices
    GrpStaic* pGrp_Ifn = NULL;
	long lGrp_Num[1] = { 0 };
    if ( !GrpInfo(pnGrp_ID, lEle_Num, 0, lGrp_Num, &pGrp_Ifn) )	{
		return false;
    }

	if ( 0 < lGrp_Num[0] )
	{
		for( lGrp = 0; lGrp < lGrp_Num[0]; lGrp++ )
		{
			// Group information
			lGrp_ID  = pGrp_Ifn[lGrp].lGrpID;
			lEle_Sta = pGrp_Ifn[lGrp].lStaID;
			lEle_End = pGrp_Ifn[lGrp].lEndID;

			if ( nGrp_Ele_Min < pGrp_Ifn[lGrp].lEleNum )
			{
				for ( lEle = lEle_Sta; lEle <= lEle_End; lEle++ )
				{
					if ( lGrp_ID == pnGrp_ID[ lEle ] )	
					{
						pnGrp_ID[ lEle ] = lGrp_Cnt;
					}
				}
				lGrp_Cnt++;
			}
			else
			{
				for ( lEle = lEle_Sta; lEle <= lEle_End; lEle++ )
				{
					if ( lGrp_ID == pnGrp_ID[ lEle ] )	
					{
						pnGrp_ID[ lEle ] = INSIGNIFICANCE;
					}
				}
			}
		} // end of for
	} // end of if
	else
	{
		for ( lEle = 0; lEle <= lEle_Num; lEle++ )
		{
			pnGrp_ID[ lEle ] = INSIGNIFICANCE;
		}
	}

	delete pGrp_Ifn;
	pGrp_Ifn = NULL;
	return true;
}

/*******************************************************************
* 
* Ele_Num_Max :			To find the group with the maximum elements
* 
* Inputs:
*	pnGrp_ID:			Group ID array, n-by-1
*	lEle_Num:			Input array dimension, n
* 
* Outputs:
*	plGrp_ID_Max:		Group ID
*                       ID of the group which is with the maximum element
*	plGrp_Ele_Num_Max:	Group element number
*   plGrp_Ele_Sta_End:  Start/end position of the group elements
*
* Revision history:
* 03/2012			    version 1.0, created by Xinlian Liang
*
* Remarks:
*
*******************************************************************/
bool Ele_Num_Max(int* pnGrp_ID, const long lEle_Num, long* plGrp_ID_Max, long* plGrp_Ele_Num_Max, long* plGrp_Ele_Sta_End)
{
	// Group information
	long lGrp = 0;
	// Group statistices
    GrpStaic* pGrp_Ifn = NULL;
	long lGrp_Num[1] = { 0 };
    if ( !GrpInfo(pnGrp_ID, lEle_Num, 0, lGrp_Num, &pGrp_Ifn) )	{
		return false;
    }

	if ( 0 < lGrp_Num[0] )
	{
		long* plGrp_Ele_Num = NULL;
		if ( !New_Long( &plGrp_Ele_Num, lGrp_Num[0] ) )	{
			return false;
		}
		for( lGrp = 0; lGrp < lGrp_Num[0]; lGrp++ )
		{
			*(plGrp_Ele_Num+lGrp) = pGrp_Ifn[lGrp].lEleNum;
		}
		*plGrp_ID_Max		   = ArrayMax( plGrp_Ele_Num, lGrp_Num[0] );
		*plGrp_Ele_Num_Max     = pGrp_Ifn[ *plGrp_ID_Max ].lEleNum;
		*plGrp_Ele_Sta_End     = pGrp_Ifn[ *plGrp_ID_Max ].lStaID;
		*(plGrp_Ele_Sta_End+1) = pGrp_Ifn[ *plGrp_ID_Max ].lEndID;
		Del_Long(&plGrp_Ele_Num);
	} // end of if
	else
	{
		*plGrp_ID_Max      = INSIGNIFICANCE;
		*plGrp_Ele_Num_Max = INSIGNIFICANCE;
	}

	delete pGrp_Ifn;
	pGrp_Ifn = NULL;
	return true;
}


// seg by row
bool rowSeg(double* &dataArray,
            int Horizon_SCAN, int N_SCAN,
            double dDif_Th, int nSch_Stp, int lGrp_Ele_Min,
            cv::Mat &pointTypeMat)
{
    // parameters of row and column
    long lEle_Num = N_SCAN * Horizon_SCAN;
    long lPfl_Num = N_SCAN; // cloumn NUM
    long lPfl_Ele_Num = Horizon_SCAN; // row NUM
    
    // parameters of segmentation
    long lEle = 0; 
    long lOft = 0;
    
    int* pnGrp_ID = NULL;
    if ( !New_Int(&pnGrp_ID, lEle_Num) )    return false;
    // segmentation
    for ( lEle = 0; lEle < lPfl_Num; lEle++ )
    {
        // the offest is "lPfl_Ele_Num" in each iteration
        lOft = lEle * lPfl_Ele_Num;
        Profile( (dataArray+lOft), lPfl_Ele_Num, dDif_Th, nSch_Stp, (pnGrp_ID+lOft) );
    }

    // grp ele num
    if ( 0 < lGrp_Ele_Min )
    {
        for ( lEle = 0; lEle < lPfl_Num; lEle++ )
        {
            lOft = lEle * lPfl_Ele_Num;

            long lMaxID = ArrayMax( (pnGrp_ID+lOft), lPfl_Ele_Num );
            if(pnGrp_ID[lMaxID+lOft] != INSIGNIFICANCE && pnGrp_ID[lMaxID+lOft] != 0)
            {   
                // std::cout << "here is good***: " << pnGrp_ID[lMaxID+lOft] << "  ID: " << lMaxID << std::endl;
                Grp_ID_Ord( (pnGrp_ID+lOft), lPfl_Ele_Num, lGrp_Ele_Min );
            }
        }            
    }

    // reserve trunk points
    int index;
    for(int columnIdn = 0; columnIdn<N_SCAN; columnIdn++){
        for(int rowIdn = 0; rowIdn<Horizon_SCAN; rowIdn++){
            index = rowIdn + columnIdn * Horizon_SCAN;
            if(pnGrp_ID[index] >= 0){
                pointTypeMat.at<float>(rowIdn, columnIdn) = 1;
            }
        }
    }
    // std::cout << "sta: row---" << sta << std::endl;
    return true;
}
// seg by column
bool colSeg(double* &dataArray,
            int Horizon_SCAN, int N_SCAN,
            double dDif_Th, int nSch_Stp, int lGrp_Ele_Min,
            cv::Mat &pointTypeMat)
{
    long lEle_Num = N_SCAN * Horizon_SCAN;
    long lPfl_Num = Horizon_SCAN; // row NUM
    long lPfl_Ele_Num = N_SCAN; // cloumn NUM
    
    long lEle = 0; 
    long lOft = 0;
    int* pnGrp_ID = NULL;
    if ( !New_Int(&pnGrp_ID, lEle_Num) )	return false;

    // segmentation
    for ( lEle = 0; lEle < lPfl_Num; lEle++ )
    {
        lOft = lEle * lPfl_Ele_Num;
        Profile( (dataArray+lOft), lPfl_Ele_Num, dDif_Th, nSch_Stp, (pnGrp_ID+lOft) );
    }

    // reserve the size is bigger than lGrp_Ele_Min 
    if ( 0 < lGrp_Ele_Min )
    {
        for ( lEle = 0; lEle < lPfl_Num; lEle++ )
        {
            lOft = lEle * lPfl_Ele_Num;
            
            long lMaxID = ArrayMax( (pnGrp_ID+lOft), lPfl_Ele_Num );
            if(pnGrp_ID[lMaxID+lOft] != INSIGNIFICANCE && pnGrp_ID[lMaxID+lOft] != 0)
            {
                Grp_ID_Ord( (pnGrp_ID+lOft), lPfl_Ele_Num, lGrp_Ele_Min );
            }     
        }
    }

    // reserve ground points
    int index;
    int sta = 0;
    for(int rowIdn=0; rowIdn<Horizon_SCAN; rowIdn++){
        for(int columnIdn=0; columnIdn<N_SCAN; columnIdn++){
            index = columnIdn + rowIdn * N_SCAN;
            if(pnGrp_ID[index] >= 0){
                pointTypeMat.at<float>(rowIdn, columnIdn) = 1;
            }
        }
    }
    // std::cout << "sta: " << sta << std::endl;
    return true;
}

// seg the points by row or column
void segPoints(cv::Mat &pointMat,
                int Horizon_SCAN, int N_SCAN,
                cv::Mat &pointTypeMat,
                ConfigSetting config_setting)
{
    double dDif_Th = config_setting.dDif_Th;
    int nSch_Stp = config_setting.nSch_Stp; 
    int lGrp_Ele_Min = config_setting.lGrp_Ele_Min;

    cv::Mat pointTypeMat_ROW(Horizon_SCAN, N_SCAN, CV_32F, cv::Scalar::all(INSIGNIFICANCE));
    cv::Mat pointTypeMat_COL(Horizon_SCAN, N_SCAN, CV_32F, cv::Scalar::all(INSIGNIFICANCE));
    
    // re-array the data by row
    double *dataArray_ROW = new double[N_SCAN*Horizon_SCAN];
    for(int j=0; j<N_SCAN; j++)
    {
        for(int i=0; i<Horizon_SCAN; i++)
        {
            dataArray_ROW[i+j*Horizon_SCAN] = pointMat.at<float>(i, j);
        }
    }

    // re-array the data by column
    double *dataArray_COL = new double[N_SCAN*Horizon_SCAN];
    for(int i=0; i<Horizon_SCAN; i++)
    {
        for(int j=0; j<N_SCAN; j++)
        {
            dataArray_COL[j+i*N_SCAN] = pointMat.at<float>(i, j);
        }
    }

    int isRow_Or_Column = config_setting.isRow_Or_Column;
    // select the seg way
    if(isRow_Or_Column == 0)
    { 
        rowSeg(dataArray_ROW, Horizon_SCAN, N_SCAN,
                dDif_Th, nSch_Stp, lGrp_Ele_Min,
                pointTypeMat_ROW);
        pointTypeMat = pointTypeMat_ROW;      
    }
    else if(isRow_Or_Column == 1)
    {
        colSeg(dataArray_COL, Horizon_SCAN, N_SCAN,
                dDif_Th, nSch_Stp, lGrp_Ele_Min,
                pointTypeMat_COL);
        pointTypeMat = pointTypeMat_COL;
    }
    else if(isRow_Or_Column == 2)
    {
        rowSeg(dataArray_ROW, Horizon_SCAN, N_SCAN,
                dDif_Th, nSch_Stp, lGrp_Ele_Min,
                pointTypeMat_ROW);
        colSeg(dataArray_COL, Horizon_SCAN, N_SCAN,
                dDif_Th, nSch_Stp, lGrp_Ele_Min,
                pointTypeMat_COL);
        int count=0;
        for(int i=0; i<Horizon_SCAN; i++)
        {
            for(int j=0; j<N_SCAN; j++)
            {
                if(pointTypeMat_ROW.at<float>(i,j)==1 || pointTypeMat_COL.at<float>(i,j)==1)
                {
                    pointTypeMat.at<float>(i,j)=1;
                    count++;
                }
            }
        }
        std::cout << "The count num: " << count << std::endl;
    }
}

// resize the pix value to 0-1
void resizePixVal(cv::Mat& matData)
{
    double minValue, maxValue;    // 最大值，最小值
    cv::Point  minIdx, maxIdx;    // 最小值坐标，最大值坐标     
    cv::minMaxLoc(matData, &minValue, &maxValue, &minIdx, &maxIdx);
    std::cout << "minValue: " << minValue 
            << ", maxValue: " << maxValue << std::endl;
    if(maxValue == 1)
    {
        for(int i=0; i<matData.rows; i++){
            for(int j=0; j<matData.cols; j++){
                matData.at<float>(i, j) = matData.at<float>(i, j)*255;
            }
        }
    }
    else
    {
        for(int i=0; i<matData.rows; i++){
            for(int j=0; j<matData.cols; j++){
                if(matData.at<float>(i, j) != 0 && matData.at<float>(i, j) != INSIGNIFICANCE)
                {
                    // 1 - matData.at<float>(i, j)/maxValue
                    matData.at<float>(i, j) = 1 - matData.at<float>(i, j)/maxValue;
                    matData.at<float>(i, j) = matData.at<float>(i, j)*255;
                }
            }
        }        
    }      
}
