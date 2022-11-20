#include "HL_MRF.h"

int main(int argc,char*argv[])
{
	// parameter setting
	std::string ss;
	std::string so;

	ss = argv[1];							   // input directory
	so = argv[2];							   // output directory
	int block_size = atoi(argv[3]);			   // block size for hierarchical regisration (default: 5)
	float downsample_size = atof(argv[4]);	   // downsample size of input TLS scans for coarse registration (default: 0.1 m)
	float downsample_size_icp = atof(argv[5]); // distance of searching corresponding point in icp (default: the same as ${downsample_size})
	float lum_iter = atof(argv[6]);			   // number of iteration for lum optimization (default: 3)
	int t_MCS = atoi(argv[7]);				   // threshold of MCS (default: 10)
	int number_of_threads = atoi(argv[8]);	   // number of threads you use (default: 0)
	bool visualize_each_block = atoi(argv[9]); // visualize each scan-block (default: 0)

	WHU::HL_MRF mf;
	mf.setBlockSize(block_size);
	mf.setPlyPath(ss);
	mf.setCoarseRgistration(CoarseRegistration::GROR);
	mf.setDownsampleSize(downsample_size);
	mf.setDownsampleSizeForICP(downsample_size_icp);
	mf.setLumIterations(lum_iter);
	mf.visualizeEachBlock(visualize_each_block);
	mf.setMaximumConsensusSet(t_MCS);
	mf.setNumberOfThreads(number_of_threads);
	mf.setOutDir(so);
	mf.performMultiviewRegistration();
	// mf.performShapeGrowingRegistration();
	return 1;
}