#ifndef GDB_H_
#define GDB_H_

#include <stdlib.h>
#include <stdint.h>
#include <string>

/* Eigen Dependencies */
#include <Eigen/Eigen>
#include <Eigen/StdVector>

template <typename T>
	void prettyPrint(T matrix, std::string name)
	{
		size_t num_rows = matrix.rows();
		size_t num_cols = matrix.cols();

		//Matrix header name
		printf(name.data());
		printf("Size: [%d,%d]\n", num_rows, num_cols);

		//Create column labels
		printf("\t");
		for (size_t col = 0; col < num_cols; col++)
			printf("[%d]\t", col);
		printf("\n\n");
	

		//Create row data
		double data = 0.0;
		for (size_t row = 0; row < num_rows; row++)
		{
			printf("[%d]\t", row);
			for (size_t col = 0; col < num_cols; col++)
			{
				data = matrix(row, col);
				if (data >= 0.0)
					printf("+%.2f\t", data);
				else
					printf("%.2f\t", data);

			}
			printf("\n\n\n");
		}

	}


#endif /* !GDB_H_ */