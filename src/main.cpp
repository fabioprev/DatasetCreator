#include "DatasetCreator.h"

using namespace std;

int main(int argc, char** argv)
{
	if (argc != 2)
	{
		ERR("Wrong syntax. Usage: ./DatasetCreator <dataset-path>" << endl);
		
		return -1;
	}
	
	DatasetCreator datasetCreator(argv[1]);
	
	datasetCreator.exec();
	
	return 0;
}
