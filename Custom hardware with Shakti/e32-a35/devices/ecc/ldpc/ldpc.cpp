// C++ implementation to read 
// file word by word 
#include <stdio.h> 
//#include <stdlib>
#include <iostream>
#include <fstream>
#include <string>
using namespace std; 

// driver code 
int main() 
{ 
//=======================================================================================================
// to extract contents of file(having the h matrix) into array (int) form

	int h[7][135],i,j,M[3][135];
	i=j=0;
	
	// filestream variable file 
	fstream file; 
	string word, t, q, filename; 

	// filename of the file 
	filename = "file_no4cycle_col3.txt"; 

	// opening file 
	file.open(filename.c_str()); 
	
	// extracting words from the file 
	while (file >> word && i<7) 
	{ 
		if(word == "0")
		{	
			//cout<<"I: "<<i<<" J: "<<j<<" "<<word<<endl;
			h[i][j] = 0;
			//cout<<"I: "<<i<<" J: "<<j<<" h[i][j]: "<<h[i][j]<<endl;
			j++;
			if(j==135)	
			{
				i++;
				j = 0;
			}
		
		}
		else if(word == "1")
		{
				//cout<<"I: "<<i<<" J: "<<j<<" "<<word<<endl;
			h[i][j] = 1;
			//cout<<"I: "<<i<<" J: "<<j<<" h[i][j]: "<<h[i][j]<<endl;
			j++;
			if(j==135)	
			{
				i++;
				j = 0;	
			}
		
		}
		else	continue;
	} 
	
//=======================================================================================================	

// to generate h matrix after the specified order of g matrix
	int order[135] = { 0,   1,  27,   4,   6,   5,   7,   3,   8,   9,  10,  11,  12,  13,  14,  15,  16,  17,  18,  19,
  20,  21,  22,  23,  24,  25,  26,   2,  28,  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,  39,
  40,  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,
  60,  61,  62,  63,  64,  65,  66,  67,  68,  69,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79,
  80,  81,  82,  83,  84,  85,  86,  87,  88,  89,  90,  91,  92,  93,  94,  95,  96,  97,  98,  99,
 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119,
 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134};
	int h_matrix_new[7][135];
	for(j=0;j<135;j++)
	{
		for(i=0;i<7;i++)		
			h_matrix_new[i][j] = h[i][order[j]];
	}
	
// to print the ordered h matrix
	
	for(i=0;i<7;i++)
	{
		cout<<endl;
		for(j=0;j<135;j++)	
			cout<<h_matrix_new[i][j]<<" ";
	}

//===========================================================================================
	
	// to get the eqn number of each bit
	
	int flag = 0;
	for(j=0;j<135;j++)
	{
		for(i=0;i<7;i++)
			if(h_matrix_new [i][j]==1)
			{
				M[flag][j] = i;
				flag++;	
			}	
		flag = 0;
	}
	cout<<endl;
	// printing the bit number and equation numbers.
	for(j=0;j<135;j++)
	{
		cout<<"Bit"<<134-j<<" equations: ";
		for(i=0;i<3;i++)
			cout<<M[i][j]<<" ,";
		cout<<endl;	
	}
	
//=======================================================================================================

// to extract codeword 
	int c[135];
	// filestream variable file 
	fstream filec; 
	string wordc, tc, qc, filenamec; 

	// filename of the file 
	filenamec = "filec.txt"; 

	// opening file 
	filec.open(filenamec.c_str()); 
	
	i=0;
	// extracting words from the file 
	while (filec >> wordc && i<135) 
	{ 
		if(wordc == "0")
		{	
			//cout<<"I: "<<i<<" J: "<<j<<" "<<word<<endl;
			c[i]= 0;
			//cout<<"I: "<<i<<" J: "<<j<<" h[i][j]: "<<h[i][j]<<endl;
			i++;
		}
		else if(wordc == "1")
		{
				//cout<<"I: "<<i<<" J: "<<j<<" "<<word<<endl;
			c[i] = 1;
			//cout<<"I: "<<i<<" J: "<<j<<" h[i][j]: "<<h[i][j]<<endl;
			i++;
		}
		else	continue;
	} 
	cout<<endl;
	for(i=0;i<135;i++)	cout<<c[i]<<" ";
//===========================================================================================
//
//	// to get syndrome;
	cout<<endl<<"syndrome"<<endl;
	int s[7];
	for(i=0;i<7;i++)
		s[i] =0;
	for(i=0;i<7;i++)	
	{
		for(j=0;j<135;j++)
		{
			//if(h_matrix_new[i][j] == 1)
			{
				s[i] = (s[i]+h_matrix_new[i][j]*c[j])%2;
			}
		}
	}
	
	cout<<endl;
	for(i=0;i<7;i++)
		cout<<s[i]<<" ";
	return 0; 


} 

