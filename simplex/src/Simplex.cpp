#include "Simplex.hpp"




MatrixXf lexicRowsOrder (MatrixXf Mat)
{
	// Lexicographic ordered matrices
	auto lex_Mat(Mat);
	auto temporary(Mat);
	VectorXf difference;
	// Indexes
	unsigned int i,j,k;

	// Checking columns and ordering
	for (i = 0; i < lex_Mat.rows()-1; i++)		//The i-th row compare to the j-th others
		for (j = i+1; j < lex_Mat.rows(); j++)
		{
			difference = lex_Mat.row(i) - lex_Mat.row(j);

			//The first non zero element
			k= 0;
			while(difference(k)==0 && k < difference.size()-1)
				k++;


			if(difference(k)>0)
			{
				// Switching columns
				for (k = 0; k < lex_Mat.cols(); k++)
				{
					temporary(i,k) = lex_Mat(i,k);
					lex_Mat(i,k) = lex_Mat(j,k);
					lex_Mat(j,k) = temporary(i,k);
				}
			}
		}
	return lex_Mat;
}

MatrixXf lexicColsOrder (MatrixXf Mat, bool direction)
{
	// Lexicographic ordered matrices
	auto lex_Mat(Mat);
	auto temporary(Mat);
	VectorXf v1,v2,difference;
	// Indexes
	unsigned int i,j,k;

	// Checking columns and ordering
	for (i = 0; i < lex_Mat.cols()-1; i++)		//The i-th row compare to the j-th others
		for (j = i+1; j < lex_Mat.cols(); j++)
		{
			difference = lex_Mat.col(i) - lex_Mat.col(j);

			//The first non zero element
			k= 0;
			while(difference(k)==0 && k < difference.size()-1)
				k++;

			if(direction){
				if(difference(k)>0)
				{
					// Switching columns
					for (k = 0; k < lex_Mat.rows(); k++)
					{
						temporary(k,i) = lex_Mat(k,i);
						lex_Mat(k,i) = lex_Mat(k,j);
						lex_Mat(k,j) = temporary(k,i);
					}
				}
			}
			else{
				if(difference(k)<0)
				{
					// Switching columns
					for (k = 0; k < lex_Mat.rows(); k++)
					{
						temporary(k,i) = lex_Mat(k,i);
						lex_Mat(k,i) = lex_Mat(k,j);
						lex_Mat(k,j) = temporary(k,i);
					}
				}
			}
		}
	return lex_Mat;
}

MatrixXf baseInitialization ()
{
	MatrixXf B = MatrixXf::Zero(ARTIFICIAL_PARAM + 1,ARTIFICIAL_PARAM);

	for (unsigned int i = 0; i < B.rows(); i++)
		for (unsigned int j = 0; j < B.cols(); j++)
		{
			if(i == j && i < ARTIFICIAL_PARAM)
				B(i,j) = 1;
			else if(i >= ARTIFICIAL_PARAM)
				B(i,j) = M;
		}

	return B;
}

MatrixXf H_generation(VectorXf constraints)
{
	MatrixXf H(ARTIFICIAL_PARAM+1,PARAMETERS);				//Matrix H
	// iterator
	unsigned int i,j,k;

	for (i = 0; i < H.rows(); i++)
	{
		k = 1;
		for (j = 0; j < H.cols(); j++)
		{
			if (i < AGENTS)
			{
				if (j >=  i*AGENTS && j <  (i + 1)*AGENTS)
					H(i,j) = 1;
				else
					H(i,j) = 0;
			}
			else if ( i >= AGENTS && i < ARTIFICIAL_PARAM)
			{
				if (i - AGENTS == j)
					H(i,j) = 1;
				else if (j == AGENTS*k + i - AGENTS){
					H(i,j) = 1;
					k = k +1;
				}
				else
					H(i,j) = 0;	
			}
			else if(i >= ARTIFICIAL_PARAM)
				H(i,j) = constraints[j];
		}
	}
	return H;
	
}

MatrixXf simplex (MatrixXf H, MatrixXf B, MatrixXf* P)
{
	// Lexicographic ordered matrices
	// Matrices
	MatrixXf A_b = MatrixXf::Zero(ARTIFICIAL_PARAM,ARTIFICIAL_PARAM);
	MatrixXf inv_A_b = MatrixXf::Zero(ARTIFICIAL_PARAM,ARTIFICIAL_PARAM);
	MatrixXf L = MatrixXf::Zero(ARTIFICIAL_PARAM,ARTIFICIAL_PARAM+1);
	MatrixXf L_trans = MatrixXf::Zero(ARTIFICIAL_PARAM,ARTIFICIAL_PARAM+1);
	MatrixXf L_ordered = MatrixXf::Zero(ARTIFICIAL_PARAM,ARTIFICIAL_PARAM+1);
	MatrixXf N = MatrixXf::Zero(ARTIFICIAL_PARAM,ARTIFICIAL_PARAM+1);
	//Vectors
	VectorXf D = VectorXf::Zero(ARTIFICIAL_PARAM);
	VectorXf c_b = VectorXf::Zero(ARTIFICIAL_PARAM);
	VectorXf e, e_b, r_e(ARTIFICIAL_PARAM+1), l_min(ARTIFICIAL_PARAM+1);
	VectorXf decisions = VectorXf::Ones(ARTIFICIAL_PARAM);
	bool flag1,flag2;
	// Indexes
	unsigned int i,j,k, min =0, counter = 0, permanent_col;

	while(counter < H.cols()){
		for ( i= 0; i < H.cols(); i++)
		{
			// e is the working colum
			e = H.col(i);

			// Check if e is one of my permanet cols
			flag2 = false;
			for(j = 0; j < P->cols(); j++)
				if(e.isApprox(P->col(j)))
				{
					flag2 = true; 
					permanent_col = j;
				}

			// I am taking the upper square part of the Base in order to be invertible
			A_b = B.block(0,0,ARTIFICIAL_PARAM,ARTIFICIAL_PARAM);

			// Only the elements related to the base
			e_b = e.segment(0,ARTIFICIAL_PARAM);
			c_b = B.row(ARTIFICIAL_PARAM);
			//	The inverse of A_b
			inv_A_b = A_b.inverse();
			// The first element
			r_e(0) = e(ARTIFICIAL_PARAM) - e_b.transpose()*inv_A_b.transpose()*c_b;
			// The others used to prevent the unboundness of solution
			r_e.segment(1,ARTIFICIAL_PARAM) = e_b.transpose()*inv_A_b.transpose();

			// Lexsort
			k = 0;
			while (r_e(k) == 0)
				k = k +1;

			// Lexicographically ratio test
			if (r_e(k) < 0)
			{
				counter = 0;
				//Initialization of minimum
				l_min = RowVectorXf::Zero(ARTIFICIAL_PARAM+1);

				// Numerator and denominator for the lexmin
				D = inv_A_b*e_b;
				N.col(0) = inv_A_b*decisions;
				N.block(0,1,ARTIFICIAL_PARAM,ARTIFICIAL_PARAM) = inv_A_b;

				// selection of the j-th row of base B
				// check if the denominator is different greater than zero
				for (j = 0; j< L.rows(); j++)
					if(D(j) > 0)
					{
						auto numerator = N.row(j);
						auto denominator = D(j);
						L.row(j) = numerator/denominator;
					}
					else
						L.row(j) = RowVectorXf::Zero(ARTIFICIAL_PARAM+1);

				// Find the lexmin, lexsort the matrix by transpose 2 times
				L_ordered = lexicRowsOrder(L);

				// Store the minimum
				flag1 = false;
				k = 0, j=0;
				while (!flag1)
				{
					if (!L_ordered.row(k).isApprox(RowVectorXf::Zero(ARTIFICIAL_PARAM+1)))
					{
						while(!L_ordered(k,j))
							j++;
						if(L_ordered(k,j) > 0)
							flag1 = true;
					}
					k++;
					j = 0;
				}
				k = k-1;
				// If it is not a zero vector you can apply the change in the base
				if( k < ARTIFICIAL_PARAM+1)
				{
					l_min = L_ordered.row(k);

					// Store the index of the minimum
					for(j = 0; j < L.rows(); j++)
						if (L_ordered.row(k).isApprox(L.row(j)))
							min = j;

					// Update the base B and the permanet cols
					if(flag2)
						P->col(permanent_col) = B.col(min);

					B.col(min) = e;
				}
			}
			else
				counter++;
		}
	}
	return B;
}

