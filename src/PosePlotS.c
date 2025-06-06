/*-----------------------------------------------------------------------

	PosePlot.c
	
	Hayashi & Satake Kinematics Calculation system Shell
	
				Copylight 2016- DRINCO Project 

-------------------------------------------------------------------------*/





#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/shm.h>


#include "HS.h"
#include "Sspace.h"

extern SSPACE *GetNewSpace( int key, int Ssize );
extern SSPACE *GetSpaceAddr( int key );
extern SSPACE *GenModelFromFileS( int key, char *fname );

int main( int argc, char *argv[] ){

	JOINT 	*Joints ;
	SSPACE 	*sp;
	int 	num;


	if(argc<2){
		printf("arguments: <KEY>\n");
			return 0;
	}
	//Jointデータの読み込み
	sp = GetSpaceAddr( atoi(argv[1]) );
	Joints = (JOINT*)(sp+1);
	//グラフ表示用位置データの出力
	OutPutPointsData(stdout, Joints, sp->base, NULL);
	shmdt(sp);
	return 0;
}

