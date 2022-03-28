//C++ implementation of RM codes
/* We are considering the code length n = 256 bits. Since we want to correct exactly one error, d = 3. But substituting in the inequality (refer RM codes pdf), we get r to be 6. This means that k = 247 bits and number of parity bits is 9 bits. Thus for 512 bytes (4096 bits) of data around 18 bytes of spare area will be used which is well within the maximum capability. We are sending 247 nbits of data every clock cycle. Thus in 16 clock cycles, 247*16 = 3952 bits will be sent. Remaining 144 bits will be sent in the 17 th clock cycle. 103 bits of zeroes need to appended so that the message will result in 247 bit long data. These zeroes can later be removed at the decoder. Hence, the remaining spare area can be used to store these zeroes. 

Thus we will be considering RM(6,8) as hte generator matrix and RM(1,8) as the dual or in other words the parity check matrix of RM(6,8). Note that RM(m-2,m) is same as an extended Hamming code. Hence the syndrome directly result in the bit position of the error.* (In our case it will be the complement because of the way we have generated the generator and parity check matrix).
*/
#include <stdio.h> 
//#include <stdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
using namespace std; 
// This function calculates the transpose of h matrix so that it can be multiplied with the codeword of 256 bits.
void transpose(int h[9][256],int ht[256][9])
{
	int i,j;
	for(i=0;i<9;i++)
		for(j=0;j<256;j++)
			ht[j][i] = h[i][j];
}
//This function performs matrix multiplication of generator matrix and h transpose matrix to check if they result in a zero matrix. This guarantees if we picked the dual correctly or not. You may skip the understanding of this function wothout loss of track of the rest of the code. It is done in mod2 arithmetic.
void mat_mul(int gen[247][256],int ht[256][9])
{
	int i,j,k,ans[247][9];
	for(i=0;i<247;i++)
	{
		
		for(j=0;j<9;j++)
		{
			ans[i][j]=0;
			for(k=0;k<256;k++)
				//%2 as we are doing mod2 arithmetic
				ans[i][j] = (ans[i][j] + gen[i][k]*ht[k][j])%2;
		
		}
	}
	//displaying the h transpose matrix
//	cout<<" H transpose"<<endl<<endl;
//	for(i=0;i<256;i++)
//	{
//		cout<<endl;
//		for(j=0;j<9;j++)	
//			cout<<ans[i][j];
//	}
}
//This function generates G(1,n) matrix. Based on the "RM Codes" pdf, we are starting with 1s and not 0s. It is important to keep this in mind as it will affect the way the syndrome is printed elsewhere.
void gnn(int g[][256],int n)
{
	int i,j,k;
	for(i=0;i<n+1;i++)
	{
		k=0;
		for(j=0;j<pow(2,n);j++)	
		{
			//The first row is an all 1 row.
			if(i==0)
				g[i][j] = 1;
			else
			{
				//We need to have 2^i 1s and 0s each as a repeating pattern. Hence, there are (2^n)/(2^i) blocks of 1s and 0s each. At every column index which is a multiple of 2^(n-i), the pattern changes from 1 to 0 or 0 to 1. At this point, we increment k. Whenever k is even or odd after incrementation, the pattern is a set of 2^i 0s or 1s respectively. You may work it out for yourself on pen and paper to understand it better.
				if(j==k*pow(2,(n-i)))	k++;
				//cout<<k<<endl;
				g[i][j] = k%2;
							
			}
		}
	}
}
//This function performs multiplication of 2 rows to yield degree 2 monomial rows.  
void mul2(int r1[], int r2[], int ans[])
{
	int i,j;
	for(i=0;i<256;i++)
	{
		//It is sufficient to just multiply the row elements as AND operation is same multiplication in integer for our purposes.
		ans[i] = r1[i]*r2[i];
	}
}
//This function performs multiplication of 3 rows to yield degree 3 monomial rows.  
void mul3(int r1[], int r2[], int r3[], int ans[])
{
	int i;
	for(i=0;i<256;i++)
		ans[i] = r1[i]*r2[i]*r3[i];
}
//This function performs multiplication of 4 rows to yield degree 4 monomial rows.  
void mul4(int r1[], int r2[], int r3[], int r4[], int ans[])
{
	int i;
	for(i=0;i<256;i++)
		ans[i] = r1[i]*r2[i]*r3[i]*r4[i];
}
//This function performs multiplication of 5 rows to yield degree 5 monomial rows.  
void mul5(int r1[], int r2[], int r3[], int r4[], int r5[], int ans[])
{
	int i;
	for(i=0;i<256;i++)
		ans[i] = r1[i]*r2[i]*r3[i]*r4[i]*r5[i];
}
//This function multiplies message (247 bits) with generator matrix (247x256) to yield a codeword (256 bits).
void encode(int message[247],int gen[247][256],int codeword[256])
{
	int i,j;
	cout<<endl<<endl<<"codeword: "<<endl;
	for(j=0;j<256;j++)
	{
		codeword[j] = 0;
		for(i=0;i<247;i++)
		{
			codeword[j] = (codeword[j] + message[i]*gen[i][j])%2;	
		}
		cout<<codeword[j];
	}
}
//This function is to perform decoding in case of single error received data. 
/*
The decoding is done as follows:
1. Calculate the syndrome. 
2. If the syndrome is not 9'b111111111, then there is either a single or double error. More errors cannot be detected.
3. In case of single error, the decimal value of the syndrome is the index at which the error has occured. In our case, we are considering a BSC as the channel. Hence, we must flip the bit at the index specified by syndrome.
*/
void decode(int codeword[256], int syndrome[9])
{
	int pos,i;
	//Initialise pos to 0. If no syndrome has occured, pos will change to 511 and no bit is flipped. 
	pos = 0;
	//For loop to get the decimal value of syndrome.
	for(i=0;i<9;i++)
	{
		//Note that it 8-i because we have stored the MSB in the lower index and LSB in higher index. Keep this in mind while reading the code in cpp and bsv. In bsv MSB is in the higher index and LSB is in lower index. Thus slight differences will be clearer when you keep the above to information in mind. 
		pos += (pow(2,8-i))*syndrome[i];
	//	cout<<pos<<" ";
	}
	//If there is a single error, then pos will be from 0 to 255.
	if(pos<256)
	{
	
	//	cout<<"pos"<<pos<<" ";
	//Bit flipping is essentially adding 1 to the exisitng bit and taking mod2, i.e XOR ing.
	codeword[pos] = (codeword[pos] + 1)%2;
	//Displaying the location of error.
	//	cout<<endl<<endl<<"POS: "<<pos+1<<endl<<endl;
	}
	//Displaying the decoded cipher
	cout<<"Decoded: "<<endl<<endl;
	for(i=0;i<256;i++)
		cout<<codeword[i];
}
//This function calculates the syndrome by multiplying the codeword with h transpose. 
void syndrome(int codeword[256], int ht[256][9])
{
	int i,j,syndrome[9];
	//Displaying the codeword
	cout<<endl<<endl<<"Codeword: ";
	for(i=0;i<256;i++)	cout<<codeword[i];
	//Displaying the syndrome as and when calculated.
	cout<<endl<<endl<<"Syndrome: ";
	for(j=0;j<9;j++)
	{
		syndrome[j] = 0;
		for(i=0;i<256;i++)
		{
			syndrome[j] = (syndrome[j] + codeword[i]*ht[i][j])%2;
		}
		//We are taking the complement of each bit after every j loop because of the way our generator matrix was constructed and the special case of RM(6,8). We started with 1s in each row and not 0s. Also, the RM(m-2,m) is an extended Hamming code version. Putting the two together, it can be seen that we will have to take the complement. You may comment the below line and see that the bit location = complement of syndrome. Thus without the below line, the syndrome will be zero for a valid codeword. With the below line, it will be 9'b111111111 for a valid codeword.
		syndrome[j] = (syndrome[j] + 1)%2;
		cout<<syndrome[j];
		
	}
	//Calling the function decoded to perform decoding and correcting single bit error.
	decode(codeword,syndrome);
}

int main() 
{ 
	int gen[247][256],h[9][256],ht[256][9],i,j,k;
	//A G(m,m) is matrix of having all posible degrees of monomials. Thus its size will be 2^m x 2^m. 
	//The sizes of G(4,6), G(5,7), G(6,7) can be calculated by hand.
	int g44[16][256],g55[32][256],g66[64][256],g46[57][64],g57[120][128],g67[127][128];
	int codeword[256];
	//Randomly generated message.
	int message[247] = {0,0,0,0,1,0,0,0,0,0,0,1,0,0,1,0,1,0,1,1,0,1,0,1,1,1,1,1,1,1,1,0,0,1,0,0,0,0,0,1,0,0,1,1,1,0,0,0,1,1,0,0,0,1,0,0,1,0,1,0,1,1,1,1,0,0,1,1,0,0,1,0,0,0,1,0,1,1,0,1,1,1,0,0,0,1,0,1,0,1,1,1,0,0,0,0,0,1,1,1,1,0,0,0,0,1,0,1,0,1,1,1,1,0,1,0,1,1,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,1,1,0,0,1,1,0,1,0,1,0,0,1,1,0,1,0,1,1,0,0,1,1,0,1,1,0,1,1,1,1,1,0,1,1,1,0,0,0,0,1,1,1,0,0,1,1,0,1,0,0,0,0,0,1,1,0,0,1,1,0,0,1,1,0,0,0,0,0,0,1,1,1,1,0,0,0,1,0,0,0,0,1,0,0,0,1,1,1,1,0,0,1,0,1,0,1,0,0,1,1,0,1,0,0,0,0,1,0,1,0,1,0,0};
	
//===================================================================================
	//Generating G(4,4) matrix.

	//First generate G(1,4).
	gnn(g44,4);
	
	//Generate G(2,4) by considering all degree 2 monomials in x1,x2,x3,x4. The list of all possible degree 2 monomial was obtained using the "comination" cpp code. Store the obtained rows in subsequent rows from 5 to 10 th index.
	mul2(g44[1],g44[2],g44[5]);
	mul2(g44[1],g44[3],g44[6]);
	mul2(g44[1],g44[4],g44[7]);
	mul2(g44[2],g44[3],g44[8]);
	mul2(g44[2],g44[4],g44[9]);
	mul2(g44[3],g44[4],g44[10]);
	
	//Generate G(3,4) by considering all degree 3 monomials in x1,x2,x3,x4 and store in subsequent rows. 
	mul3(g44[1],g44[2],g44[3],g44[11]);
	mul3(g44[1],g44[2],g44[4],g44[12]);
	mul3(g44[1],g44[3],g44[4],g44[13]);
	mul3(g44[2],g44[3],g44[4],g44[14]);
	
	//Generate G(4,4) by considering all degree 4 monomials in x1,x2,x3,x4 and store in subsequent row. 
	mul4(g44[1],g44[2],g44[3],g44[4],g44[15]);	
	
// Displaying G(4,4) matrix.
//	cout<<endl<<endl<<"G(4,4)";
//	for(i=0;i<16;i++)
//	{
//		cout<<endl;
//		for(j=0;j<16;j++)
//			cout<<g44[i][j]<<" ";
//	}
//===================================================================================	
	// Generating G(5,5) matrix.
	
	//First generate G(1,5) matrix.
	gnn(g55,5);
		
	//Generate G(2,5) by considering all degree 2 monomials in x1,x2,x3,x4,x5 and store in subsequent rows. 
	mul2(g55[1],g55[2],g55[6]);
	mul2(g55[1],g55[3],g55[7]);
	mul2(g55[1],g55[4],g55[8]);
	mul2(g55[1],g55[5],g55[9]);
	mul2(g55[2],g55[3],g55[10]);
	mul2(g55[2],g55[4],g55[11]);
	mul2(g55[2],g55[5],g55[12]);
	mul2(g55[3],g55[4],g55[13]);
	mul2(g55[3],g55[5],g55[14]);
	mul2(g55[4],g55[5],g55[15]);
	
	//Generate G(3,5) by considering all degree 3 monomials in x1,x2,x3,x4,x5 and store in subsequent rows. 
	mul3(g55[1],g55[2],g55[3],g55[16]);
	mul3(g55[1],g55[2],g55[4],g55[17]);
	mul3(g55[1],g55[2],g55[5],g55[18]);
	mul3(g55[1],g55[3],g55[4],g55[19]);
	mul3(g55[1],g55[3],g55[5],g55[20]);
	mul3(g55[1],g55[4],g55[5],g55[21]);
	mul3(g55[2],g55[3],g55[4],g55[22]);
	mul3(g55[2],g55[3],g55[5],g55[23]);
	mul3(g55[2],g55[4],g55[5],g55[24]);
	mul3(g55[3],g55[4],g55[5],g55[25]);
	
	//Generate G(4,5) by considering all degree 4 monomials in x1,x2,x3,x4,x5 and store in subsequent rows. 
	mul4(g55[1],g55[2],g55[3],g55[4],g55[26]);
	mul4(g55[1],g55[2],g55[3],g55[5],g55[27]);
  	mul4(g55[1],g55[2],g55[4],g55[5],g55[28]);
	mul4(g55[1],g55[3],g55[4],g55[5],g55[29]);
	mul4(g55[2],g55[3],g55[4],g55[5],g55[30]);
	
	//Generate G(5,5) by considering all degree 5 monomials in x1,x2,x3,x4,x5 and store in subsequent rows. 
	mul5(g55[1],g55[2],g55[3],g55[4],g55[5],g55[31]);

// Displaying G(5,5) matrix.	
//	cout<<endl<<endl<<"G(5,5)"<<endl;
//	for(i=0;i<32;i++)
//	{
//			cout<<endl;
//		for(j=0;j<32;j++)
//			cout<<g55[i][j]<<" ";
//	}
//===================================================================================	
	//Generating G(6,6) matrix
	
	//First generate G(1,6) matrix.
	gnn(g66,6);
	
	//Generate G(2,6) by considering all degree 2 monomials in x1,x2,x3,x4,x5,x6 and store in subsequent rows. 
	mul2(g66[1],g66[2],g66[7]);
	mul2(g66[1],g66[3],g66[8]);
	mul2(g66[1],g66[4],g66[9]);
	mul2(g66[1],g66[5],g66[10]);
	mul2(g66[1],g66[6],g66[11]);
	mul2(g66[2],g66[3],g66[12]);
	mul2(g66[2],g66[4],g66[13]);
	mul2(g66[2],g66[5],g66[14]);
	mul2(g66[2],g66[6],g66[15]);
	mul2(g66[3],g66[4],g66[16]);
	mul2(g66[3],g66[5],g66[17]);
	mul2(g66[3],g66[6],g66[18]);
	mul2(g66[4],g66[5],g66[19]);
	mul2(g66[4],g66[6],g66[20]);
	mul2(g66[5],g66[6],g66[21]);
	
	//Generate G(3,6) by considering all degree 3 monomials in x1,x2,x3,x4,x5,x6 and store in subsequent rows. 
	mul3(g66[1],g66[2],g66[3],g66[22]);
	mul3(g66[1],g66[2],g66[4],g66[23]);
	mul3(g66[1],g66[2],g66[5],g66[24]);
	mul3(g66[1],g66[2],g66[6],g66[25]);
	mul3(g66[1],g66[3],g66[4],g66[26]);
	mul3(g66[1],g66[3],g66[5],g66[27]);
	mul3(g66[1],g66[3],g66[6],g66[28]);
	mul3(g66[1],g66[4],g66[5],g66[29]);
	mul3(g66[1],g66[4],g66[6],g66[30]);
	mul3(g66[1],g66[5],g66[6],g66[31]);
	mul3(g66[2],g66[3],g66[4],g66[32]);
	mul3(g66[2],g66[3],g66[5],g66[33]);
	mul3(g66[2],g66[3],g66[6],g66[34]);
	mul3(g66[2],g66[4],g66[5],g66[35]);
	mul3(g66[2],g66[4],g66[6],g66[36]);
	mul3(g66[2],g66[5],g66[6],g66[37]);
	mul3(g66[3],g66[4],g66[5],g66[38]);
	mul3(g66[3],g66[4],g66[6],g66[39]);
	mul3(g66[3],g66[5],g66[6],g66[40]);
	mul3(g66[4],g66[5],g66[6],g66[41]);
	
	//Generate G(4,6) by considering all degree 4 monomials in x1,x2,x3,x4,x5,x6 and store in subsequent rows. 
	mul4(g66[1],g66[2],g66[3],g66[4],g66[42]);
	mul4(g66[1],g66[2],g66[3],g66[5],g66[43]);
	mul4(g66[1],g66[2],g66[3],g66[6],g66[44]);
	mul4(g66[1],g66[2],g66[4],g66[5],g66[45]);
	mul4(g66[1],g66[2],g66[4],g66[6],g66[46]);
	mul4(g66[1],g66[2],g66[5],g66[6],g66[47]);
	mul4(g66[1],g66[3],g66[4],g66[5],g66[48]);	
	mul4(g66[1],g66[3],g66[4],g66[6],g66[49]);
	mul4(g66[1],g66[3],g66[5],g66[6],g66[50]);
	mul4(g66[1],g66[4],g66[5],g66[6],g66[51]);
	mul4(g66[2],g66[3],g66[4],g66[5],g66[52]);
	mul4(g66[2],g66[3],g66[4],g66[6],g66[53]);
	mul4(g66[2],g66[3],g66[5],g66[6],g66[54]);
	mul4(g66[2],g66[4],g66[5],g66[6],g66[55]);
	mul4(g66[3],g66[4],g66[5],g66[6],g66[56]);
	
	//Generate G(5,6) by considering all degree 5 monomials in x1,x2,x3,x4,x5,x6 and store in subsequent rows. 
	mul5(g66[1],g66[2],g66[3],g66[4],g66[5],g66[57]);
	mul5(g66[1],g66[2],g66[3],g66[4],g66[6],g66[58]);
	mul5(g66[1],g66[2],g66[3],g66[5],g66[6],g66[59]);
	mul5(g66[1],g66[2],g66[4],g66[5],g66[6],g66[60]);
	mul5(g66[1],g66[3],g66[4],g66[5],g66[6],g66[61]);
	mul5(g66[2],g66[3],g66[4],g66[5],g66[6],g66[62]);
	
	//Generate G(6,6) by considering all degree 6 monomials in x1,x2,x3,x4,x5,x6 and store in subsequent row.
	//As expected, the last row of G(m,m) matrix will be 1 followed by (2^m - 1) 0s. 
	g66[63][0] = 1;
	for(j=1;j<64;j++)
		g66[63][j] = 0;
	
// Displaying G(6,6) matrix
//	cout<<endl<<endl<<"G(6,6)"<<endl;
//	
//	for(i=0;i<64;i++)
//	{
//		cout<<endl;
//		for(j=0;j<64;j++)
//			cout<<g66[i][j]<<" ";
//	}
//===================================================================================

// G(r,m) = | G(r,m-1)		G(r,m-1) |
//			| G(r-1,m-1)		0    |
// The zero sub matrix is on the bottom right because we are starting with 2^i 1s and then 2^i 0s. If we started the other way, then it would look like this:
// G(r,m) = | G(r,m-1)	     G(r,m-1) |
//			| 	0      		G(r-1,m-1)|

//===================================================================================

	//Generating G(4,6) matrix.

	//g46 = | g45	g45	|
	//		| g35	0	|
	
	//The range of i and j can be calculated manually.
	for(i=0;i<31;i++)
		for(j=0;j<32;j++)
		{
			g46[i][j] = g55[i][j];
			//We are considering j+32 because we are just replicating g45 in the left and right top sub matrices of g46.
			g46[i][j+32] = g55[i][j];
		}
	
	//G(3,5) is same as the first 27 rows of G(5,5)		
	for(i=31;i<57;i++)
	{
	//	cout<<endl;
			for(j=0;j<32;j++)
		{
			// i-31 is used because we need to acess the first 27 rows of g55 and fill it from the 31st row index of g46.
			g46[i][j] = g55[i-31][j];
			//cout<<g55[i-31][j];
		}
	}
	
//NOTE: The rest of the elements are zero by default. In case while displaying this isnt the case, additional lines of codes may have to be written to make them to 0.

// Displaying G(4,6)
//	cout<<endl<<endl<<"G(4,6)"<<endl;
//	for(i=0;i<57;i++)
//	{
//		cout<<endl;
//		for(j=0;j<64;j++)
//			cout<<g46[i][j];
//	}
//===================================================================================	

	// Generating G(5,7)
	
	//g57 = |	g56		g56	|
	//		|	g46		0	|
	
	//The range of i and j can be calculated manually.	
	for(i=0;i<63;i++)
		for(j=0;j<64;j++)
		{
			g57[i][j] = g66[i][j];
			//We are considering j+64 because we are just replicating g56 in the left and right top sub matrices of g57.
			g57[i][j+64] = g66[i][j];
		}
	
	//Filling bottom left sub matrix of G(5,7) with G(4,6).
	for(i=63;i<120;i++)
		for(j=0;j<64;j++)
			// i-63 is used because we need to acess the 57 rows of g46 and fill it from the 63rd row index of g57.
			g57[i][j] = g66[i-63][j];
			
//NOTE: The rest of the elements are zero by default. In case while displaying this isnt the case, additional lines of codes may have to be written to make them to 0.			

// Displaying G(5,7)
//	cout<<endl<<endl<<"G(5,7)"<<endl;
//	for(i=0;i<120;i++)
//	{
//		cout<<endl;
//		for(j=0;j<128;j++)
//			cout<<g57[i][j];
//	}
//===================================================================================

	// Generating G(6,7) matrix.
	
	//g67 = |	g66		g66	|
	//		|	g56		0	|
	
	//The range of i and j can be calculated manually.
	for(i=0;i<64;i++)
		for(j=0;j<64;j++)
		{
			g67[i][j] = g66[i][j];
			//We are considering j+64 because we are just replicating g66 in the left and right top sub matrices of g67.
			g67[i][j+64] = g66[i][j];
		}
	
	//Filling bottom left sub matrix of G(6,7) with G(5,6).
	// G(5,6) is same as the G(6,6) without the last row.
	for(i=64;i<127;i++)
		for(j=0;j<64;j++)
			// i-64 is used because we need to access the first 63 rows of G(6,6) and fill it from the 64th row index of G(6,7). 
			g67[i][j] = g66[i-64][j];

//NOTE: The rest of the elements are zero by default. In case while displaying this isnt the case, additional lines of codes may have to be written to make them to 0.			

// Displaying G(6,7)
//	cout<<endl<<endl<<"G(6,7)"<<endl;
//	for(i=0;i<127;i++)
//	{
//		cout<<endl;
//		for(j=0;j<128;j++)
//			cout<<g67[i][j];
//	}
//===================================================================================
	
	//Generating H matrix = G(1,8).
	//g18=h (9x256)
	gnn(h,8);
	//Taking transpose of H matrix and storing it in ht (256x9).
	transpose(h,ht);
	
	// Displaying H matrix.
	//cout<<endl<<endl<<"H"<<endl<<endl;
	//for(i=0;i<9;i++)
	//{
	// 	cout<<endl;
	//	for(j=0;j<256;j++)
	// 		cout<<h[i][j];
	//}
	
	// Displaying H transpose matrix.
	//cout<<endl<<endl<<"Ht"<<endl<<endl;
	//for(i=0;i<256;i++)
	//	{
	//		cout<<endl;
	//		for(j=0;j<9;j++)
	//			cout<<h[i][j];
	//	}	
//===================================================================================

	//Generating G(6,8) matrix = Generator Matrix
	
	//g68=gen = |	g67		g67	|
	//			|	g57		0	|
	
	//The range of i and j can be caluclated manually.
	for(i=0;i<127;i++)
		for(j=0;j<128;j++)
		{
			gen[i][j] = g67[i][j];
			//We are considering j+128 because we are just replicating g67 in the left and right top sub matrices of g68.
			gen[i][j+128] = g67[i][j];
		}
	//Filling bottom left sub matrix of G(6,8) with G(5,7).
	for(i=127;i<247;i++)
		for(j=0;j<128;j++)
		{
			// i-64 is used because we need to access the 120 rows of G(5,7) and fill it from the 127th row index of G(6,8). 
			gen[i][j] = g57[i-127][j];
			//Making the columns from 128 to 255 zero.
			gen[i][j+128] = 0;
		}
	
	// This will display the result of generatore matrix times transpose of parity check matrix. It has to be zero. If it is not, then there is a mistake in the generation of generator or parity check matrix.
	//mat_mul(gen,ht);
	
	// Displaying G(6,8) matrix = generator matrix
	//cout<<endl<<endl<<"G(6,8)"<<endl;
	//for(i=0;i<247;i++)
	//{
	//	cout<<endl;
	//	for(j=0;j<256;j++)
	//		cout<<gen[i][j];
	//}
//===================================================================================

	//message = 247 bits
	// codeword = 256 bits
	cout<<endl<<endl<<"Message: "<<endl;
	for(i=0;i<247;i++)
	{
		//message[i] = m[i];
		cout<<message[i];
	}

// Encoding the message	
	encode(message,gen,codeword);

// Purposely creating error at the 4th bit of codeword.
	codeword[3] = 1;
// Calculating syndrome and correcting error.
	syndrome(codeword,ht);

//===================================================================================
/*
It has been mentioned that the syndrome can be directly linked with position of single bit error. In our case, bit position of error = complement of syndrome. This was possible because of the fact that G(6,8) is an extended Hamming code and our G and H matrices started with 2^i 1s and then 2^i 0s. For any other RM code which doesn't become an extended Hamming code, this may not be the case. 

A general approach would be see what is the value of syndrome for each single bit error. We know that, received data = valid codeword + error. Thus r times h transpose  = c times h transpose + e times h transpose. 
			c times h transpose  = zero matrix. 
	   Thus r times h transpose = e times h transpose
r = received data which may or may not have an error, c = codeword, e = error.
So, find the syndromes of all possible one bit error 256'b1000...0, 256'b010....0, 256'b0010...0, ..., 256'b000...0100, 256'b000...010 and 256'b00...001 and form a look up table consisting of syndromes and correctable errors

Thus, after calculating the syndrome of received data, simply search it on the look up table. It will tell you the error corresponding to that syndrome. Once, you know the error, go to that specific bit positiona nd flip the bit.

Also note that number of possible single bit error is 256 (length of codeword). But possible values of syndrome = 512 (2^9). Thus, the rest of the syndromes correspond to double errors. Number of possible double bit errors is 256 C 2 (256 choose 2). Definitely multiple double errors will result in the same syndrome. Thus when the syndrome lies in the values corresponding to double bit errors, we will only be able to detect and say that two errors have occurred. However, we will not be able to correct them.
*/

/*
In the following lines of code, we are generating all possible 1 bit errors can calcualting the syndrome. Once, we get the syndrome, it can be stored in a look up table and be used while decoding.
*/

//	int error[256];

// Initialise the bits to zero.
//	for(i=0;i<256;i++)
//	{
//		error[i] = 0;
//	}

// Get the syndrome of no error vector. In our case it will be 9'b111111111 (as we are taking complement of syndrome).
//	syndrome(error,ht);

// Generating all possible single bit error by right shifting by 1 at every iteration.
//	for(i=0;i<256;i++)
//	{
		// Make the ith bit to be 1.
//		error[i] = 1;
		//Calculate the syndrome of error.
//		syndrome(error,ht);
		// Before incrementing i, make that bit to be zero (right shifting).
//		error[i] = 0;
//	}
	return 0;
}

