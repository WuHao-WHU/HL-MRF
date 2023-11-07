#include "../include/HL_MRF.h"

WHU::HL_MRF::HL_MRF():
	voxel_size(0.2),
	voxel_size_icp(0.1),
	use_cyclic_constraints(1),
	use_LUM(1),
	translation_accuracy(0.5),
	rotation_accuracy(0.087),
	approx_overlap(0.3),
	part_num(5),
	check_blcok(false),
	bigloop(0),
	all_count(0),
	use_pairs(false),
	max_consensus_set(10),
	num_threads(0)
{

}
WHU::HL_MRF::~HL_MRF()
{
}

void WHU::HL_MRF::init()
{
	if (bigloop == 0)
	{
		part_loop = static_cast<int>(files.size() / part_num) + 1;
		part_num_rest = static_cast<int>(files.size() % part_num);
		all_count = 0;
	}
	else {
		//uncomment to continue divide
		/*part_loop = static_cast<int>(part_clouds.size() / part_num) + 1;
		part_num_rest = static_cast<int>(part_clouds.size() % part_num);
		all_count = 0;*/

		part_loop = 1;
		part_num_rest = part_clouds.size();
		all_count = 0;

	}
	if (part_num_rest == 0)
	{
		part_loop--;
	}

	if (use_pairs)
	{
		part_loop = 1;
	}
}

void WHU::HL_MRF::blockPartition(int block_id)
{
	
	pairs.clear();
	filename.str("");
	nr_scans = 0;
	if (bigloop == 0)
	{
		filename << "/block" << block_id;
	}
	else {
		filename << "/Final_aligned_scan" << block_id ;
	}
	if (part_loop > 1 && block_id < part_loop - 1)
	{
		for (int i = 0; i < part_num; i++)
		{
			for (int j = i + 1; j < part_num; j++)
			{

				pairs.push_back(std::make_pair(i, j));
				//overlap_score.push_back(1 - all_dists[i][j]);

			}
			nr_scans++;
		}
	}

	if (block_id == part_loop - 1 && part_num_rest != 0)
	{
		for (int i = 0; i < part_num_rest; i++)
		{
			for (int j = i + 1; j < part_num_rest; j++)
			{

				pairs.push_back(std::make_pair(i, j));
				//overlap_score.push_back(1 - all_dists[i][j]);

			}
			nr_scans++;
		}
	}

	if (block_id == part_loop - 1 && part_num_rest == 0)
	{
		for (int i = 0; i < part_num; i++)
		{
			for (int j = i + 1; j < part_num; j++)
			{

				pairs.push_back(std::make_pair(i, j));
				//overlap_score.push_back(1 - all_dists[i][j]);

			}
			nr_scans++;
		}
	}

	if (use_pairs)
	{
		//readPairs(pair_path);
		readPairs();
		nr_scans = files.size();
	}
}

void WHU::HL_MRF::transferVaribles(
	std::vector <pcl::PointCloud <PointT>::Ptr>& keypoint_clouds,
	std::vector <pcl::PointCloud<pcl::FPFHSignature33>::Ptr> &keypoint_clouds_feature,
	std::vector <pcl::PointCloud <PointT>::Ptr> &clouds,
	int block_id
)
{
	if (bigloop == 0)
	{
		keypoint_clouds.resize(nr_scans);
		keypoint_clouds_feature.resize(nr_scans);
		clouds.resize(nr_scans);
	}
	else {
		keypoint_clouds.resize(nr_scans);
		keypoint_clouds_feature.resize(nr_scans);
		clouds.resize(nr_scans);
		//if the block continue divide,load clouds at specific spots
		if (part_loop > 1 && block_id < part_loop - 1)
		{
			for (int i = 0; i < part_num; i++)
			{
				clouds[i] = std::move(part_clouds[block_id * part_num + i]);
				keypoint_clouds[i] = std::move(part_keypoint_clouds[block_id * part_num + i]);
				keypoint_clouds_feature[i] = std::move(part_keypoint_feature_clouds[block_id * part_num + i]);
			}
		}
		if (block_id == part_loop - 1 && part_num_rest == 0)
		{
			for (int i = 0; i < part_num; i++)
			{
				clouds[i] = std::move(part_clouds[block_id * part_num + i]);
				keypoint_clouds[i] = std::move(part_keypoint_clouds[block_id * part_num + i]);
				keypoint_clouds_feature[i] = std::move(part_keypoint_feature_clouds[block_id * part_num + i]);
			}
		}
		else if (block_id == part_loop - 1 && part_num_rest != 0) {
			for (int i = 0; i < part_num_rest; i++)
			{
				clouds[i] = std::move(part_clouds[block_id * part_num + i]);
				keypoint_clouds[i] = std::move(part_keypoint_clouds[block_id * part_num + i]);
				keypoint_clouds_feature[i] = std::move(part_keypoint_feature_clouds[block_id * part_num + i]);
			}
		}

	}
}

int WHU::HL_MRF::readPLYfiles()
{
	if (!boost::filesystem::exists(PLYpath))
	{
		cerr << "...path does not exists!\n";
		return 0;
	}
	if (!boost::filesystem::is_directory(PLYpath))
	{
		cerr << "...path is not a directory!\n";
		return 0;
	}
	for (boost::filesystem::directory_entry& x : boost::filesystem::directory_iterator(PLYpath))
	{
		files.push_back(x.path());
	}
	std::sort(files.begin(),files.end());
	return 1;
}

int WHU::HL_MRF::readPairs()
{
	pairs.clear();
	//pairs.resize(files.size());
	for (boost::filesystem::directory_entry& x : boost::filesystem::directory_iterator(pairs_dir))
	{
		pairs_path.push_back(x.path());
	}
	for (int i = 0; i < pairs_path.size(); i++)
	{
		char s1 = pairs_path[i].filename().string()[1];
		char s1_ = pairs_path[i].filename().string()[2];
		char s2 = pairs_path[i].filename().string()[5];
		char s2_ = pairs_path[i].filename().string()[6];
		int s1_i = s1 - '0';
		s1_i = s1_i * 10;
		int s1_i_ = s1_ - '0';

		s1_i = s1_i + s1_i_ - 1;
		int s2_i = s2 - '0';
		s2_i = s2_i * 10;
		int s2_i_ = s2_ - '0';
		s2_i = s2_i + s2_i_ - 1;
		pairs.push_back(std::make_pair(s1_i, s2_i));
	}
	return 1;
}

int WHU::HL_MRF::sampleLeafsized( pcl::PointCloud<PointT>::Ptr& cloud_in, 
	pcl::PointCloud<PointT>& cloud_out, 
	float downsample_size)
{

	pcl::PointCloud <PointT> cloud_sub;
	cloud_out.clear();
	float leafsize = downsample_size * (std::pow(static_cast <int64_t> (std::numeric_limits <int32_t>::max()) - 1, 1. / 3.) - 1);

	pcl::octree::OctreePointCloud <PointT> oct(leafsize); // new octree structure
	oct.setInputCloud(cloud_in);
	oct.defineBoundingBox();
	oct.addPointsFromInputCloud();

	pcl::VoxelGrid <PointT> vg; // new voxel grid filter
	vg.setLeafSize(downsample_size, downsample_size, downsample_size);
	vg.setInputCloud(cloud_in);

	size_t num_leaf = oct.getLeafCount();

	pcl::octree::OctreePointCloud <PointT>::LeafNodeIterator it = oct.leaf_begin(), it_e = oct.leaf_end();
	for (size_t i = 0; i < num_leaf; ++i, ++it)
	{
		pcl::IndicesPtr ids(new std::vector <int>); // extract octree leaf points
		pcl::octree::OctreePointCloud <PointT>::LeafNode* node = (pcl::octree::OctreePointCloud <PointT>::LeafNode*) * it;
		node->getContainerPtr()->getPointIndices(*ids);

		vg.setIndices(ids); // set cloud indices
		vg.filter(cloud_sub); // filter cloud

		cloud_out += cloud_sub; // add filter result
	}

	return (static_cast <int> (cloud_out.size())); // return number of points in sampled cloud
}

void WHU::HL_MRF::readPointCloud(const boost::filesystem::path& filename,
	pcl::PointCloud<PointT>::Ptr cloud
)
{
	if (!filename.extension().string().compare(".ply"))
	{
		pcl::io::loadPLYFile(filename.string(), *cloud);
		return;
	}
	if (!filename.extension().string().compare(".pcd"))
	{
		pcl::io::loadPCDFile(filename.string(), *cloud);
		return;
	}
}

void WHU::HL_MRF::preprocessing(std::vector<pcl::PointCloud<PointT>::Ptr>& keypoint_clouds, 
	std::vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr>& keypoint_clouds_feature,
	std::vector<pcl::PointCloud<PointT>::Ptr>& clouds,int block_id 
)
{

#pragma omp parallel for num_threads(num_threads) 
	for (int i = 0; i < nr_scans; i++)
	{
		cout << "Processing " << files[block_id * part_num + i].stem().string() << ":" << endl;

		// load point cloud
		cout << "..loading point cloud..";
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		readPointCloud(files[block_id * part_num + i], cloud);
		// color(cloud);
		clouds[i] = cloud;
		//cout << "ok!" << endl;

		// voxel sampel point cloud
		cout << "..apply voxel grid filter..\n";
		pcl::PointCloud <PointT>::Ptr voxel_cloud(new pcl::PointCloud <PointT>);
		pcl::PointCloud<PointT>::Ptr voxel_cloud_icp(new pcl::PointCloud<PointT>);
		sampleLeafsized(clouds[i], *voxel_cloud, voxel_size);
		sampleLeafsized(clouds[i], *voxel_cloud_icp, voxel_size_icp);
		clouds[i] = voxel_cloud_icp;
		//cout << "ok!" << endl;

		//iss
		pcl::PointCloud<PointT>::Ptr issS(new pcl::PointCloud<PointT>);
		pcl::PointIndicesPtr issIdxS(new pcl::PointIndices);
		std::cout << "extracting ISS keypoints..." << voxel_size << std::endl;
		GrorPre::issKeyPointExtration(voxel_cloud, issS, issIdxS, voxel_size);
		std::cout << "size of issS = " << issS->size() << std::endl;
		issS->is_dense = false;
		keypoint_clouds[i] = issS;


		//fpfh
		std::cout << "computing fpfh..." << std::endl << std::endl;;

		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhS(new pcl::PointCloud<pcl::FPFHSignature33>());
		GrorPre::fpfhComputation(voxel_cloud, voxel_size, issIdxS, fpfhS);
		keypoint_clouds_feature[i] = fpfhS;
		cout << "ok!\n";


	}

}

void WHU::HL_MRF::coarseRgistration(std::vector<pcl::PointCloud<PointT>::Ptr>& keypoint_clouds, 
	std::vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr>& keypoint_clouds_feature,
	std::vector<pcl::PointCloud<PointT>::Ptr>& clouds, 
	std::vector<int>& pairs_best_count,
	std::vector <pcl::registration::MatchingCandidate, Eigen::aligned_allocator<pcl::registration::MatchingCandidate>>& candidate_matches
)
{
	//compare with best count
	//std::vector<double> pairs_RMSE(nr_pairs);
	//std::vector<double> pairs_MSAC(nr_pairs);
	for (int i = 0; i < nr_pairs;i++)
	{
		cout << "-";
	}
	cout << "\n";
	if (method == CoarseRegistration::GROR)
	{
#pragma omp parallel for num_threads(num_threads)
		for (int i = 0; i < nr_pairs; i++)
		{
			int n_optimal = 800; //optimal selection number
			const int& src = pairs[i].first;
			const int& tgt = pairs[i].second;

			//cout << "Matching keypoints of " << files[src].stem().string() << " and " << files[tgt].stem().string() << ".." << std::flush;
			int maxCorr = 5;
			pcl::CorrespondencesPtr corr(new pcl::Correspondences);
			std::vector<int> corrNOS, corrNOT;

			GrorPre::correspondenceSearching(keypoint_clouds_feature[src], keypoint_clouds_feature[tgt], *corr, maxCorr, corrNOS, corrNOT);
			//std::cout << "NO. corr = " << corr->size() << std::endl;

			pcl::registration::GRORInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, float> obor;
			pcl::PointCloud<pcl::PointXYZ>::Ptr pcs(new pcl::PointCloud<pcl::PointXYZ>);

			pcl::PointCloud<pcl::PointXYZ>::Ptr temp_src(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr temp_tgt(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud<PointT, pcl::PointXYZ>(*keypoint_clouds[src], *temp_src);
			pcl::copyPointCloud<PointT, pcl::PointXYZ>(*keypoint_clouds[tgt], *temp_tgt);
			obor.setInputSource(temp_src);
			obor.setInputTarget(temp_tgt);
			obor.setResolution(voxel_size);
			obor.setOptimalSelectionNumber(n_optimal);
			obor.setNumberOfThreads(1);
			obor.setInputCorrespondences(corr);
			obor.setDelta(voxel_size);
			obor.align(*pcs);
			

			pcl::registration::MatchingCandidate result;
			result.transformation = obor.getFinalTransformation();
			result.fitness_score = obor.getMSAC();
			pairs_best_count[i] = obor.getBestCount();
			//pairs_MSAC[i] = obor.getMSAC();
			//double RMSE = getRMSE(clouds[src], clouds[tgt], obor.getFinalTransformation(), voxel_size_icp);
			//pairs_RMSE[i] = RMSE;

			////debug
			//cout << src << "--" << tgt << " best count:" << obor.getBestCount() << endl;
			//cout << src << "--" << tgt << " fitness score:" << obor.getMSAC() << endl;
			// if (candidates.size() == 0)
			// {
			// 	pcl::registration::MatchingCandidate temp;
			// 	temp.transformation = Eigen::Matrix4f::Zero();
			// 	temp.fitness_score = 1;
			// 	candidates.push_back(temp);
			// }
#pragma omp critical 
			{
				candidate_matches[i]=result;
				cout << "*";
				cout.clear();
			}

			//cout << "ok" << endl;
		}
		cout << "\n";
		//cout<<"matches size: "<<candidate_matches.size();
	}
	
// 	if (method == CoarseRegistration::SDOFGR)
// 	{
// #pragma omp parallel for num_threads(num_threads)
// 		for (int i = 0; i < nr_pairs; i++)
// 		{
// 			const int& src = pairs[i].first;
// 			const int& tgt = pairs[i].second;

// 			cout << "Matching keypoints of " << files[src].stem().string() << " and " << files[tgt].stem().string() << ".." << std::flush;
// 			//match features
// 			std::cout << "matching correspodences..." << std::endl;
// 			int maxCorr = 3;
// 			std::vector<GenIn::corrTab> corr;
// 			std::vector<int> corrNOS, corrNOT;

// 			GenIn::corrComp(keypoint_clouds_feature[src], keypoint_clouds_feature[tgt], corr, maxCorr, corrNOS, corrNOT);
// 			std::cout << "NO. corr = " << corr.size() << std::endl;
// 			ImprovedGlobalRegistration6DoF global_registration_6dof;
// 			global_registration_6dof.SetDistanceThreshold(2.0 * voxel_size);
// 			global_registration_6dof.SetOutputLog(log_file, true);
// 			pcl::PointCloud<pcl::PointXYZ>::Ptr temp_src(new pcl::PointCloud<pcl::PointXYZ>);
// 			pcl::PointCloud<pcl::PointXYZ>::Ptr temp_tgt(new pcl::PointCloud<pcl::PointXYZ>);
// 			pcl::copyPointCloud<PointT, pcl::PointXYZ>(*keypoint_clouds[src], *temp_src);
// 			pcl::copyPointCloud<PointT, pcl::PointXYZ>(*keypoint_clouds[tgt], *temp_tgt);
// 			global_registration_6dof.SetCorrespondence(corr, temp_src, temp_tgt);
// 			global_registration_6dof.SetThreadNumber(1);
// 			global_registration_6dof.setDelta(voxel_size);


// 			std::vector <pcl::registration::MatchingCandidate, Eigen::aligned_allocator<pcl::registration::MatchingCandidate>> candidates;
// 			pcl::registration::MatchingCandidate result;
// 			float score = 0;
// 			auto res = global_registration_6dof.GetRegisrationResult(score);
// 			result.transformation = std::get<0>(res);
// 			result.fitness_score = score;
// 			candidates.push_back(result);
// 			pairs_best_count[i] = std::get<1>(res);
// 			cout << src << "--" << tgt << " best count:" << std::get<1>(res) << endl;
// 			cout << src << "--" << tgt << " fitness score:" << score << endl;
// 			if (candidates.size() == 0)
// 			{
// 				pcl::registration::MatchingCandidate temp;
// 				temp.transformation = Eigen::Matrix4f::Zero();
// 				temp.fitness_score = 1;
// 				candidates.push_back(temp);
// 			}
// 			candidate_matches[i] = candidates;
// 			cout << "ok" << endl;
// 		}
// 	}

	/*for (int i = 0; i < nr_pairs; i++)
	{
		cout << "RMSE:";
		cout << pairs_RMSE[i]<<endl;
	}
	for (int i = 0; i < nr_pairs; i++)
	{
		cout << "MSAC:";
		cout << pairs_MSAC[i] << endl;
	}
	for (int i = 0; i < nr_pairs; i++)
	{
		cout << "best count:";
		cout << pairs_best_count[i] << endl;
	}*/
}

void WHU::HL_MRF::globalCoarseRegistration(
	std::set<int>& rejected_pairs_,
	std::vector <pcl::registration::MatchingCandidate, Eigen::aligned_allocator<pcl::registration::MatchingCandidate>>& candidate_matches,
	std::vector<int> &pairs_best_count,
	int nr_scans
)
{
	//std::vector < std::vector<std::pair<int, bool>>> ho_Constraints;/**  invaild loop constraints */
	//std::vector < std::vector<std::pair<int, bool>>> ho_Constraints_true;/**  vaild loop constraints */
	if (nr_scans >= 3 && !use_pairs )
	{
		rejected_pairs_.clear();
		std::cout<<"--> loop-based coarse regisration. \n";

		//Loop-coarse registration method. 
		ScanGraphInference sgi;
		// typedef opengm::LazyFlipper <opengm::GraphicalModel <float, opengm::Adder>, opengm::Minimizer> Inference;
		// pcl::registration::MultiTransformationOptimization <Inference, opengm::ExplicitFunction <float> > optimizer;
		// optimizer.setPointCloudPairs(pairs);
		// optimizer.setLabelsOfNodes(candidate_matches);
		// optimizer.setInitNrOfLabels(1);
		// optimizer.setTranslationAccuracy(rotation_accuracy);
		// optimizer.setRotationAccuracy(translation_accuracy);
		// optimizer.apply(matches);
		// optimizer.getInvaildConstraints(ho_Constraints);
		// optimizer.getvaildConstraints(ho_Constraints_true);
		sgi.setScanPairs(pairs);
		sgi.setMatchingCandidates(candidate_matches);
		sgi.setMCS(pairs_best_count);
		sgi.setRotationAccuracy(rotation_accuracy);
		sgi.setTranslationAccuracy(translation_accuracy);
		rejected_pairs_ = sgi.inference(3,10);

		std::cout<<"IES size: "<<rejected_pairs_.size()<<"\n";
		//perform consistency check

		// for (int i = 0; i < matches.size(); i++)
		// {
		// 	if (candidate_matches[i].size() > 0)
		// 		matches[i] = candidate_matches[i][0];

		// }
		//cout << "vaild constratints size " << ho_Constraints_true.size() << endl;
		//cout << "invaild constratints size " << ho_Constraints.size() << endl;


	// 

	// 	if (use_cyclic_constraints)
	// 	{
	// 		std::set<int> true_cycle;
	// 		for (int i = 0; i < ho_Constraints_true.size(); i++)
	// 		{
	// 			for (int j = 0; j < ho_Constraints_true[i].size(); j++)
	// 			{
	// 				true_cycle.insert(ho_Constraints_true[i][j].first);
	// 			}
	// 		}
	// 		cout << "using cyclic constratint" << endl;
	// 		for (int i = 0; i < ho_Constraints.size(); i++)
	// 		{
	// 			int remove = 0;
	// 			for (int j = 0; j < ho_Constraints[i].size(); j++)
	// 			{

	// 				auto it = std::find(true_cycle.begin(), true_cycle.end(), ho_Constraints[i][j].first);
	// 				if (pairs_best_count[ho_Constraints[i][j].first] < max_consensus_set && it == true_cycle.end())
	// 				{
	// 					rejected_pairs_.push_back(ho_Constraints[i][j].first);
	// 					cout << "pairs" << pairs[ho_Constraints[i][j].first].first << "--" << pairs[ho_Constraints[i][j].first].second << " is rejected, due to the best_count: " << pairs_best_count[ho_Constraints[i][j].first] << "<" << max_consensus_set << endl;
	// 					remove++;
	// 				}
	// 			}
	// 			if (remove == 0 && bigloop == 0)
	// 			{
	// 				cout << "pairs are all rejected, because it's invaild but no transformation is removed\n";
	// 				for (int j = 0; j < ho_Constraints[i].size(); j++)
	// 				{
	// 					auto it = std::find(true_cycle.begin(), true_cycle.end(), ho_Constraints[i][j].first);
	// 					if (it == true_cycle.end())
	// 					{
	// 						rejected_pairs_.push_back(ho_Constraints[i][j].first);
	// 						cout << "pairs" << pairs[ho_Constraints[i][j].first].first << "--" << pairs[ho_Constraints[i][j].first].second << " is rejected, due to the best_count: " << pairs_best_count[ho_Constraints[i][j].first] << "<" << max_consensus_set << endl;
	// 					}
	// 				}
	// 			}
	// 		}
	// 	}
	// 	else {
	// 		
	// 		cout << "not using cyclic constratint" << endl;
	// 		for (int i = 0; i < matches.size(); i++)
	// 		{
	// 			if (pairs_best_count[i] < max_consensus_set)
	// 			{
	// 				rejected_pairs_.push_back(i);
	// 				cout << "pairs" << i << " is reject, due to the best_count: " << pairs_best_count[i] << "<" << max_consensus_set << endl;
	// 			}
	// 		}
	// 	}

	// }
	// else {
	// 	for (int i = 0; i < nr_pairs; i++)
	// 	{
	// 		matches[i] = candidate_matches[i][0];
	// 	}
	}
}

void WHU::HL_MRF::globalFineRegistration(std::set<int>& rejected_pairs_, 
	std::vector<pcl::registration::MatchingCandidate, Eigen::aligned_allocator<pcl::registration::MatchingCandidate>>& matches,
	std::vector <pcl::PointCloud <PointT>::Ptr> &clouds,
	std::vector<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>>& poses,
	int &num_of_subtrees
)
{
	if (nr_scans >= 3)
	{
		//temporary varible
		std::vector<std::vector<int>> LUM_indices_map_;
		std::vector<std::vector<int>> LUM_indices_map_inv_;
		std::vector<std::vector<size_t>> edges;
		std::vector<int> root;
		std::vector<std::pair<int, int>> after_check_graph_pairs;
		std::vector <pcl::registration::MatchingCandidate, Eigen::aligned_allocator<pcl::registration::MatchingCandidate>> after_check_matches;
		std::vector<std::vector<pcl::Indices>> nodes_seqs;

		mstCalculation(rejected_pairs_, matches, LUM_indices_map_, LUM_indices_map_inv_, edges, root, after_check_graph_pairs, after_check_matches,num_of_subtrees);
		pairwiseICP(edges, after_check_graph_pairs, after_check_matches, num_of_subtrees, clouds);
		concatenateFineRegistration(edges, root, after_check_graph_pairs, after_check_matches, num_of_subtrees, poses, nodes_seqs);
		LUMoptimization(poses, edges, root, nodes_seqs, LUM_indices_map_, LUM_indices_map_inv_, num_of_subtrees, clouds);
	}
}

void WHU::HL_MRF::mstCalculation(std::set<int>& rejected_pairs_,
	std::vector<pcl::registration::MatchingCandidate, Eigen::aligned_allocator<pcl::registration::MatchingCandidate>>& matches,
	std::vector<std::vector<int>>& LUM_indices_map_,
	std::vector<std::vector<int>>& LUM_indices_map_inv_,
	std::vector<std::vector<size_t>> &edges,
	std::vector<int>& root,
	std::vector<std::pair<int, int>>& after_check_graph_pairs,
	std::vector <pcl::registration::MatchingCandidate, Eigen::aligned_allocator<pcl::registration::MatchingCandidate>> &after_check_matches,
	int &num_of_subtrees
)
{
	typedef boost::subgraph<boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, boost::property<boost::vertex_index_t, int>, boost::property<boost::edge_index_t, int, boost::property<boost::edge_weight_t, float>>>> Graph;
	//typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS > Graph;
	Graph G(nr_scans);

	cout << "compute MST\n";
	std::vector<float>  weight;
	for (std::size_t i = 0; i < nr_pairs; i++)
	{
		if (rejected_pairs_.size() != 0)
		{
			if (find(rejected_pairs_.begin(), rejected_pairs_.end(), i) != rejected_pairs_.end())
			{
				cout << "ignore" << i << ", it's rejected" << endl;
				continue;
			}

		}
		const int& src = pairs[i].first;
		const int& tgt = pairs[i].second;
		add_edge(src, tgt, G);
		weight.push_back(matches[i].fitness_score);
		after_check_matches.push_back(matches[i]);
		after_check_graph_pairs.push_back(std::make_pair(src, tgt));
	}
	std::cout<<"component calculation...";
	std::vector< int > component(num_vertices(G));


	num_of_subtrees = connected_components(G, &component[0]);
	edges.resize(num_of_subtrees);

	cout<<"subgraphs size: "<<num_of_subtrees;
	 std::vector<std::vector<int>> subgraph_vertices(num_of_subtrees);
    //cout << component.size();
    for (int i = 0; i < component.size(); i++)
    {
        subgraph_vertices[component[i]].push_back(i);
        //cout << component[i] << ": " << i << "\t";
    }

	// to address the vertex&edge descriptor issue (global <-> local)
	// LUM use local descriptor
	 Graph G_r(nr_scans);
	Graph* subgraphs= new Graph[num_of_subtrees];

	for (int i = 0; i < num_of_subtrees; i++)
        {
            Graph &g_s = G_r.create_subgraph();
            for (int j = 0; j < subgraph_vertices[i].size(); j++)
            {
                //cout << subgraph_vertices[i][j] << "\t";
                boost::add_vertex(subgraph_vertices[i][j], g_s);
                //cout <<v;
            }
           //cout << "number of vertex of subgraphs: " << boost::num_vertices(g_t);

            cout << "\n";
            subgraphs[i]=g_s;
        }
	
	for (std::size_t i = 0; i < after_check_graph_pairs.size(); i++)
        {
            int g_s =component[after_check_graph_pairs[i].first];
            boost::property_map<Graph, boost::edge_weight_t>::type weightmap = boost::get(boost::edge_weight, subgraphs[g_s]);
            std::size_t src = subgraphs[g_s].global_to_local(after_check_graph_pairs[i].first);
            std::size_t tgt = subgraphs[g_s].global_to_local(after_check_graph_pairs[i].second);
            //cout<<src<<"\t";
            //cout<<tgt<<"\n";
            boost::graph_traits<Graph>::edge_descriptor e;
            bool inserted;

            boost::tie(e, inserted) = boost::add_edge(src, tgt, subgraphs[g_s]);
            weightmap[e] = weight[i];
        }
	cout << "--> find MST edges: \n";

	LUM_indices_map_.resize(num_of_subtrees);
	LUM_indices_map_inv_.resize(num_of_subtrees);
	for (int i = 0; i < num_of_subtrees ; i++)
	{
		LUM_indices_map_[i].resize(nr_scans, 0);
		LUM_indices_map_inv_[i].resize(nr_scans, 0);
	}
	 std::vector<std::vector<boost::graph_traits<Graph>::vertex_descriptor>> p(num_of_subtrees);
	for(int i=0;i<num_of_subtrees;i++)
        {
			p[i].resize(boost::num_vertices(subgraphs[i]));
			if(boost::num_vertices(subgraphs[i])==1)
				continue;
            boost::prim_minimum_spanning_tree(subgraphs[i],&p[i][0]);
        }

	for (int i = 0; i < num_of_subtrees; i++)
        {
            boost::property_map<Graph, boost::edge_index_t>::type edgemap = boost::get(boost::edge_index, subgraphs[i]);
            cout<<"\tMST: " ;
            for (int j = 0; j < p[i].size(); j++)
            {
				LUM_indices_map_[i][subgraphs[i].local_to_global(j)]=j;
				LUM_indices_map_inv_[i][j]=subgraphs[i].local_to_global(j);
                if(p[i][j]==j)
				{
					cout<<"root: "<<p[i][j]<<"\n";
					root.push_back(subgraphs[i].local_to_global(j));
                    continue;
				}

                 //cout<<boost::edge(p[i][j],size_t(j),subgraphs[i]).second;
                edges[i].push_back(edgemap[boost::edge(p[i][j],size_t(j),subgraphs[i]).first]);

                std::cout << subgraphs[i].local_to_global(boost::edge(p[i][j],size_t(j),subgraphs[i]).first) << "\t";
            }
            cout<<endl;
        }
	
	delete[] subgraphs;
	//std::vector<int> count(num_of_subtrees + 1);

	//i ->global, count ->local
	// for (int i = 0; i < nr_scans; i++)
	// {

	// 	if (pred[i] == graph.numberOfEdges())
	// 	{
	// 		LUM_indices_map_inv_[component[i]][count[component[i]]] = i;
	// 		LUM_indices_map_[component[i]][i] = count[component[i]]++;
	// 		continue;
	// 	}
	// 	edges[component[i]].push_back(pred[i]);
	// 	LUM_indices_map_inv_[component[i]][count[component[i]]] = i;
	// 	LUM_indices_map_[component[i]][i] = count[component[i]]++;

	// 	cout << component[i] << ": " << pred[i] << "\t";
	// }
	
}

void WHU::HL_MRF::pairwiseICP(std::vector<std::vector<size_t>>& edges,
	std::vector<std::pair<int, int>>& after_check_graph_pairs,
	std::vector<pcl::registration::MatchingCandidate, Eigen::aligned_allocator<pcl::registration::MatchingCandidate>>& after_check_matches,
	int &num_of_subtrees,
	std::vector <pcl::PointCloud <PointT>::Ptr>& clouds
)
{
	for (int f = 0; f < num_of_subtrees ; f++)
	{
		if (edges[f].size() == 0)
			continue;
#pragma omp parallel for num_threads(num_threads)
		for (int i = 0; i < edges[f].size(); i++)
		{

			cout << "pairwise icp process ";
			cout << "edge: " << edges[f][i] << endl;
			//normal icp matching or higher level mathcing using saved indices
			if (1)
			{

				pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::PointCloud<pcl::PointXYZ>::Ptr temp2(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::copyPointCloud(*clouds[after_check_graph_pairs[edges[f][i]].first], *temp);
				pcl::copyPointCloud(*clouds[after_check_graph_pairs[edges[f][i]].second], *temp2);

				// pcl::transformPointCloud(*temp, *temp, after_check_matches[edges[f][i]].transformation);
				// CCCoreLib::ICPRegistrationTools::Parameters params;
				// params.finalOverlapRatio = approx_overlap;
				// params.maxThreadCount = 1;

				//ICP icp;
				pcl::IterativeClosestPoint<PointT, PointT> icp;
				if (!temp->empty() && !temp2->empty())
				{
					// icp.setInputCloud(temp, temp2);
					// icp.setParameters(params);
					// icp.align();
					// after_check_matches[edges[f][i]].transformation = icp.getFinalTransformation() * after_check_matches[edges[f][i]].transformation;
					icp.setInputSource(temp);
					icp.setInputTarget(temp2);
					icp.setMaxCorrespondenceDistance(voxel_size_icp);
					icp.setUseReciprocalCorrespondences(true);
					icp.setMaximumIterations(10);
					icp.setEuclideanFitnessEpsilon(10e-3);
					icp.align(*temp, after_check_matches[edges[f][i]].transformation);
					after_check_matches[edges[f][i]].transformation = icp.getFinalTransformation();
					cout << after_check_graph_pairs[edges[f][i]].first << "--" << after_check_graph_pairs[edges[f][i]].second << "\n";
					cout << after_check_matches[edges[f][i]].transformation << "\n";
					
				}
				else {
					cout << "ignore" << i << "because one of them is nullptr!";
				}
				

				//pcl::IterativeClosestPoint<PointT, PointT> icp;
				//pcl::PointCloud<PointT>::Ptr temp(new PointCloud<PointT>());
				//icp.setInputSource(clouds[after_check_graph_pairs[edges[f][i]].first]);
				//icp.setInputTarget(clouds[after_check_graph_pairs[edges[f][i]].second]);
				////icp.setInputSource(clouds[pairs[i].first]);
				////icp.setInputTarget(clouds[pairs[i].second]);
				//icp.setMaxCorrespondenceDistance(atof(argv[11]));
				//icp.setMaximumIterations(atoi(argv[9]));
				//icp.setEuclideanFitnessEpsilon(0.001);
				//icp.setUseReciprocalCorrespondences(true);
				//icp.align(*temp, after_check_matches[edges[f][i]].transformation);
				//after_check_matches[edges[f][i]].transformation = icp.getFinalTransformation();
				//cout << after_check_graph_pairs[edges[f][i]].first << "--" << after_check_graph_pairs[edges[f][i]].second << "\n";
				//cout << after_check_matches[edges[f][i]].transformation << "\n";
				//cout << "has converged ? " << icp.hasConverged() << "\n";


			}

		}
		cout << "icp complete\n";
	}
}

void WHU::HL_MRF::concatenateFineRegistration(std::vector<std::vector<size_t>>& edges,
	std::vector<int>& root,
	std::vector<std::pair<int, int>>& after_check_graph_pairs, 
	std::vector<pcl::registration::MatchingCandidate, Eigen::aligned_allocator<pcl::registration::MatchingCandidate>>& after_check_matches, 
	int &num_of_subtrees, 
	std::vector<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>>& poses, 
	std::vector<std::vector<pcl::Indices>>& nodes_seqs)
{
	for (int f = 0; f < num_of_subtrees ; f++)
	{
		std::stringstream ss;
		ss << f;
		if (edges[f].size() == 0)
		{
			std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> poses_t(nr_scans, Eigen::Matrix4f::Zero());
			poses_t[root[f]] = Eigen::Matrix4f::Identity();
			poses.push_back(poses_t);
			std::vector<pcl::Indices> nodes_seq;
			nodes_seqs.push_back(nodes_seq);
			/*ofstream ofs2(R"(D:\Programming\global_consistency\build\result\concatenate registration)" + filename.str() + "_Mst" + ss.str() + ".txt");

			for (auto& pose : poses_t)
			{
				cout << pose << endl;
				ofs2 << pose << endl;
			}
			ofs2.close();*/
			continue;
		}


		std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> Mst_trans(nr_scans - 1);
		std::vector<pcl::Indices> Mst(nr_scans);
		for (int i = 0; i < edges[f].size(); i++)
		{
			Mst[after_check_graph_pairs[edges[f][i]].first].push_back(after_check_graph_pairs[edges[f][i]].second);
			Mst[after_check_graph_pairs[edges[f][i]].second].push_back(after_check_graph_pairs[edges[f][i]].first);
		}
		std::vector<std::pair<int, bool>> map_(nr_scans * nr_scans);
		for (std::size_t i = 0; i < edges[f].size(); i++)
		{
			map_[after_check_graph_pairs[edges[f][i]].first * nr_scans + after_check_graph_pairs[edges[f][i]].second] = std::make_pair(static_cast <int> (edges[f][i]), true);
			map_[after_check_graph_pairs[edges[f][i]].second * nr_scans + after_check_graph_pairs[edges[f][i]].first] = std::make_pair(static_cast <int> (edges[f][i]), false);
		}

		std::vector<pcl::Indices> nodes_seq;
		std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> poses_t(nr_scans, Eigen::Matrix4f::Zero());
		depthfirstsearch(nr_scans, root[f], Mst, map_, nodes_seq);
		cout << "depth search complete\n";
		for (std::vector<int> item : nodes_seq)
		{
			cout << "road :";
			for (int i : item)
			{
				cout << i << "\n";
			}
			cout << "\n";
		}
		combineTransformation(nr_scans, root[f], nodes_seq, map_, after_check_matches, poses_t);
		cout << "root " << root[f] << " is the reference frame :\n";
		//ofstream ofs2(R"(D:\Programming\global_consistency\build\result\concatenate registration)" + filename.str() + "_Mst" + ss.str() + ".txt");

		for (Eigen::Matrix4f& pose : poses_t)
		{
			cout << pose << endl;
			//ofs2 << pose << endl;
		}
		//ofs2.close();
		poses.push_back(poses_t);

		nodes_seqs.push_back(nodes_seq);
	}
}

void WHU::HL_MRF::depthfirstsearch(int nr_scans, int root, std::vector<pcl::Indices>& Mst, std::vector<std::pair<int, bool>>& map_, std::vector<pcl::Indices>& nodes_seq)
{
	std::vector<bool> visited(nr_scans, false);

	pcl::Indices path(1, root);
	visited[root] = true;
	//auto leafs =Mst[root];
	std::vector<int> indices = Mst[root];
	pcl::Indices::iterator it = indices.begin();

	for (int i = 0; i < indices.size(); i++)
	{
		int n = indices[i];
		next(n, Mst, map_, nodes_seq, visited, path);
	}
}

void WHU::HL_MRF::next(int root, std::vector<pcl::Indices>& Mst, std::vector<std::pair<int, bool>>& map_, std::vector<pcl::Indices>& nodes_seq, std::vector<bool> visited, pcl::Indices path)
{
	visited[root] = true;
	path.push_back(root);
	std::vector<int> indices = Mst[root];
	pcl::Indices::iterator it = indices.begin();
	while (it != indices.end())
	{
		if (visited[*it])
			it = indices.erase(it);
		else
			it++;
	}
	if (indices.size() == 0)
	{
		nodes_seq.push_back(path);
		return;
	}

	for (int i = 0; i < indices.size(); i++)
	{
		int n = indices[i];
		next(n, Mst, map_, nodes_seq, visited, path);
	}
}

void WHU::HL_MRF::combineTransformation(int nr_scans, int root, 
	std::vector<pcl::Indices>& nodes_seq, std::vector<std::pair<int, bool>>& map_, 
	std::vector<pcl::registration::MatchingCandidate, Eigen::aligned_allocator<pcl::registration::MatchingCandidate>>& matches, 
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>& poses)
{
	Eigen::Matrix4f combin = Eigen::Matrix4f::Identity();

	poses[root] = Eigen::Matrix4f::Identity();
	for (int i = 0; i < nodes_seq.size(); i++)
	{
		combin = Eigen::Matrix4f::Identity();
		for (int j = 0; j < nodes_seq[i].size() - 1; j++)
		{
			std::pair<int,bool> edge = map_[nodes_seq[i][j] * nr_scans + nodes_seq[i][j + 1]];
			
			if (edge.second)
				combin *= inverse(matches[edge.first].transformation);
			else combin *= (matches[edge.first].transformation);
			poses[nodes_seq[i][j + 1]] = combin;
		}
	}
}

Eigen::Matrix4f WHU::HL_MRF::inverse(Eigen::Matrix4f& mat)
{
	Eigen::Matrix3f R = mat.block(0, 0, 3, 3);
	Eigen::Vector3f t = mat.block(0, 3, 3, 1);

	Eigen::Matrix4f inversed = Eigen::Matrix4f::Identity();
	inversed.block(0, 0, 3, 3) = R.transpose().block(0, 0, 3, 3);
	inversed.block(0, 3, 3, 1) = (-(R.transpose() * t)).block(0, 0, 3, 1);
	return inversed;
}

void WHU::HL_MRF::LUMoptimization(std::vector<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>>& poses,
	std::vector<std::vector<size_t>>& edges,
	std::vector<int>& root,
	std::vector<std::vector<pcl::Indices>>& nodes_seqs,
	std::vector<std::vector<int>>& LUM_indices_map_,
	std::vector<std::vector<int>>& LUM_indices_map_inv_,
	int num_of_subtrees,
	std::vector <pcl::PointCloud <PointT>::Ptr>& clouds
)
{
	if (use_LUM)
	{

		std::set<int> visited;
		std::vector< std::vector<pcl::CorrespondencesPtr>> correspondences(edges.size());

		//*********************** apply lum global fine registration ***********************//
		for (int f = 0; f < num_of_subtrees ; f++)
		{
			int q = 0;
			std::stringstream ss;
			ss << f;
			if (edges[f].size() < 2)
			{
				ofstream ofs3(output_dir + filename.str() + "_Mst" + ss.str() + ".txt");
				cout << "...saving results to files" << endl;
				for (int i = 0; i < nr_scans; i++)
				{
					if (poses[f][i].isZero())
					{
						continue;
					}
					ofs3 << "affine transformation: \n";

					ofs3 << poses[f][i];

					ofs3 << "\n";
				}
				ofs3.close();
				continue;
			}

			pcl::registration::LUM<PointT> lum;
			cout << "apply lum global matching..\n";

			lum.addPointCloud(clouds[root[f]]);
			for (int i = 0; i < nr_scans; i++)
			{
				if (i == root[f])
					continue;
				if (poses[f][i].isZero())
				{
					continue;
				}
				Eigen::Transform<float, 3, Eigen::Affine>  aff(poses[f][i]);
				Eigen::Vector6f pose;
				pcl::getTranslationAndEulerAngles(aff, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
				lum.addPointCloud(clouds[i], pose);
			}

			int iteration_ = LUM_iterations;
			for (int li = 0; li < iteration_; li++)
			{
				visited.clear();

				cout << "...get correspondences\n";
				for (int i = 0; i < nr_scans; i++)
				{
					if (i == root[f])
						continue;
					if (poses[f][i].isZero())
					{
						continue;
					}
					Eigen::Transform<float, 3, Eigen::Affine>  aff(poses[f][i]);
					Eigen::Vector6f pose;
					pcl::getTranslationAndEulerAngles(aff, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
					lum.setPose(LUM_indices_map_[f][i], pose);
				}
				std::vector<pcl::CorrespondencesPtr> correspondences_t;

				for (int i = 0; i < nodes_seqs[f].size(); i++)
				{


#pragma omp parallel for num_threads(num_threads)
					for (int j = 0; j < nodes_seqs[f][i].size() - 1; j++)
					{
						if (std::find(visited.begin(), visited.end(), nodes_seqs[f][i][j + 1]) == visited.end())
						{
							pcl::registration::CorrespondenceEstimation<PointT, PointT> correspondence_estimate;
							pcl::CorrespondencesPtr temp(new pcl::Correspondences);
							pcl::PointCloud<PointT>::Ptr temp2(new pcl::PointCloud<PointT>());
							pcl::transformPointCloud(*clouds[nodes_seqs[f][i][j + 1]], *temp2, inverse(poses[f][nodes_seqs[f][i][j]]) * poses[f][nodes_seqs[f][i][j + 1]]);
							correspondence_estimate.setInputSource(temp2);
							correspondence_estimate.setInputTarget(clouds[nodes_seqs[f][i][j]]);
							correspondence_estimate.determineReciprocalCorrespondences(*temp, voxel_size_icp);
#pragma omp critical
							{
								visited.insert(nodes_seqs[f][i][j + 1]);
								cout << i << ":" << "correspondences sizes: " << (*temp).size() << "\n";
								lum.setCorrespondences(LUM_indices_map_[f][nodes_seqs[f][i][j + 1]], LUM_indices_map_[f][nodes_seqs[f][i][j]], temp);
							}

						}
					}
				}


				lum.setMaxIterations(1);
				lum.setConvergenceThreshold(0.001);
				cout << "perform lum optimization...\n";
				lum.compute();
				pcl::PointCloud<PointT>::Ptr cloud_out(new pcl::PointCloud<PointT>());
				for (int i = 0; i < lum.getNumVertices(); i++)
				{
					poses[f][LUM_indices_map_inv_[f][i]] = lum.getTransformation(i).matrix();
				}
			}
			//				}
			//					set<int> visited;
			//					vector< vector<pcl::CorrespondencesPtr>> correspondences(edges.size());
			//					//vector< vector<pcl::CorrespondencesPtr>> inv_correspondences(edges.size());
			//					
			//					//*********************** apply lum global fine registration ***********************//
			//					for (int f = 0; f < flag + 1; f++)
			//					{
			//						
			//						int q = 0;
			//						stringstream ss;
			//						ss << f;
			//						if (edges[f].size() < 2)
			//						{
			//							ofstream ofs3(R"(D:\Programming\global_consistency\build\result\fine registration.txt)" + filename.str() + "_Mst" + ss.str() + ".txt");
			//							cout << "...saving results to files" << endl;
			//							for (int i = 0; i < nr_scans; i++)
			//							{
			//								if (poses[f][i].isZero())
			//								{
			//									continue;
			//								}
			//								ofs3 << "affine transformation: \n";
			//
			//								ofs3 << poses[f][i];
			//
			//								ofs3 << "\n";
			//							}
			//							ofs3.close();
			//							continue;
			//						}
			//
			//						//ii = edge.begin();
			//						pcl::registration::LUM<PointT> lum;
			//						cout << "apply lum global matching..\n";
			//						//  Add point clouds as vertices to the SLAM graph
			//						//lum.addPointCloud(clouds[0]);
			//						//��rootΪ�ο�֡
			//						lum.addPointCloud(clouds[root[f]]);
			//						for (int i = 0; i < nr_scans; i++)
			//						{
			//							if (i == root[f])
			//								continue;
			//							if (poses[f][i].isZero())
			//							{
			//								continue;
			//							}
			//							Eigen::Transform<float, 3, Eigen::Affine>  aff(poses[f][i]);
			//							Eigen::Vector6f pose;
			//							pcl::getTranslationAndEulerAngles(aff, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
			//							lum.addPointCloud(clouds[i], pose);
			//						}
			//						// Use your favorite pairwise correspondence estimation algorithm(s)
			//						//Add the correspondence results as edges to the SLAM graph
			//						int iteration_ = atoi(argv[10]);
			//						for (int li = 0; li < iteration_; li++)
			//						{
			//							visited.clear();
			//							vector<bool> after_check_graph_pairs_visited(after_check_graph_pairs.size(), false);
			//							for (int i = 0; i < edges.size(); i++)
			//							{
			//								if (i == f)
			//									continue;
			//								for (int j = 0; j < edges[i].size(); j++)
			//								{
			//									after_check_graph_pairs_visited[edges[i][j]] = true;
			//								}
			//							}
			//
			//							cout << "...get correspondences\n";
			//							for (int i = 0; i < nr_scans; i++)
			//							{
			//								if (i == root[f])
			//									continue;
			//								if (poses[f][i].isZero())
			//								{
			//									continue;
			//								}
			//								Eigen::Transform<float, 3, Eigen::Affine>  aff(poses[f][i]);
			//								Eigen::Vector6f pose;
			//								pcl::getTranslationAndEulerAngles(aff, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
			//								lum.setPose(LUM_indices_map_[f][i], pose);
			//							}
			//							vector<pcl::CorrespondencesPtr> correspondences_t;
			//							//#pragma omp parallel for num_threads(8)
			//							cout << "odometry case:\n";
			//
			//#pragma omp parallel for num_threads(16)
			//							for (int j = 0; j < edges[f].size(); j++)
			//							{
			//								
			//								pcl::registration::CorrespondenceEstimation<PointT, PointT> correspondence_estimate;
			//								pcl::CorrespondencesPtr temp(new pcl::Correspondences);
			//								pcl::PointCloud<PointT>::Ptr temp2(new PointCloud<PointT>());
			//								pcl::transformPointCloud(*clouds[after_check_graph_pairs[edges[f][j]].second], *temp2, inverse(poses[f][after_check_graph_pairs[edges[f][j]].first])* poses[f][after_check_graph_pairs[edges[f][j]].second]);
			//								correspondence_estimate.setInputSource(temp2);
			//								correspondence_estimate.setInputTarget(clouds[after_check_graph_pairs[edges[f][j]].first]);
			//								correspondence_estimate.determineReciprocalCorrespondences(*temp, atof(argv[11]));
			//							#pragma omp critical
			//								{
			//									//visited.insert(nodes_seqs[f][i][j + 1]);
			//									cout << j << ":" << "correspondences sizes: " << (*temp).size() << "\n";
			//									//correspondences_t.push_back(temp);
			//									lum.setCorrespondences(LUM_indices_map_[f][after_check_graph_pairs[edges[f][j]].second], LUM_indices_map_[f][after_check_graph_pairs[edges[f][j]].first], temp);
			//								}
			//								after_check_graph_pairs_visited[edges[f][j]] = true;
			//						
			//							}
			//							cout << "loop closure case:\n";
			////#pragma omp parallel for num_threads(16)
			////							for (int j = 0; j < after_check_graph_pairs_visited.size(); j++)
			////							{
			////								if (after_check_graph_pairs_visited[j] == true)
			////									continue;
			////								cout << "loop found!\n";
			////								pcl::registration::CorrespondenceEstimation<PointT, PointT> correspondence_estimate;
			////								pcl::CorrespondencesPtr temp(new pcl::Correspondences);
			////								pcl::PointCloud<PointT>::Ptr temp2(new PointCloud<PointT>());
			////								pcl::transformPointCloud(*clouds[after_check_graph_pairs[j].second], *temp2, inverse(poses[f][after_check_graph_pairs[j].first])* poses[f][after_check_graph_pairs[j].second]);
			////								correspondence_estimate.setInputSource(temp2);
			////								correspondence_estimate.setInputTarget(clouds[after_check_graph_pairs[j].first]);
			////								correspondence_estimate.determineReciprocalCorrespondences(*temp, atof(argv[11]));
			////							#pragma omp critical
			////								{
			////									//visited.insert(nodes_seqs[f][i][j + 1]);
			////									cout << j << ":" << "correspondences sizes: " << (*temp).size() << "\n";
			////									//correspondences_t.push_back(temp);
			////									lum.setCorrespondences(LUM_indices_map_[f][after_check_graph_pairs[j].second], LUM_indices_map_[f][after_check_graph_pairs[j].first], temp);
			////								}
			////								after_check_graph_pairs_visited[j] = true;
			////							}
			//
			//							//correspondences[f] = correspondences_t;
			//							/*for (int i = 0; i < nodes_seqs[f].size(); i++)
			//							{
			//								for (int j = 0; j < nodes_seqs[f][i].size() - 1; j++)
			//								{
			//									if (correspondences[f][q]->size() < 5000)
			//										continue;
			//									
			//									q++;
			//								}
			//
			//							}*/
			//							//  Change the computation parameters
			//							lum.setMaxIterations(1);
			//							lum.setConvergenceThreshold(0.001);
			//							//Perform the actual LUM computation
			//							cout << "perform lum optimization...\n";
			//							lum.compute();
			//							// Return the concatenated point cloud result
			//							pcl::PointCloud<PointT>::Ptr cloud_out(new pcl::PointCloud<PointT>());
			//							//cloud_out = lum.getConcatenatedCloud();
			//							for (int i = 0; i < lum.getNumVertices(); i++)
			//							{
			//								poses[f][LUM_indices_map_inv_[f][i]] = lum.getTransformation(i).matrix();
			//							}
			//						}
			//
			ofstream ofs3(output_dir + filename.str() + "_Mst" + ss.str() + ".txt");
			cout << "...saving results to files" << endl;
			for (int i = 0; i < lum.getNumVertices(); i++)
			{
				ofs3 << "affine transformation: \n";
				ofs3 << lum.getTransformation(i).matrix();
				poses[f][LUM_indices_map_inv_[f][i]] = lum.getTransformation(i).matrix();
				if (i == (lum.getNumVertices() - 1))
					break;
				ofs3 << "\n";
			}
			ofs3.close();

			//pcl::visualization::PCLVisualizer vis;
			////for (int i = 0; i < nr_scans; i++)
			////{
			////    stringstream ss;
			////    ss << "cloud" << i;
			////    vis.addPointCloud(clouds[i], ss.str());
			////}
			//vis.addPointCloud(cloud_out, "clouds");
			//vis.spin();
		}
	}
}

void WHU::HL_MRF::solveGlobalPose()
{
	for (int i = Hierarchical_block_poses.size()-1; i >0 ; i--)
	{
		int count = 0;
		for (int j = 0; j < Hierarchical_block_poses[i].size(); j++)
		{
			for (int k = 0; k < Hierarchical_block_poses[i][j].size(); k++)
			{
				for (int m = 0; m < Hierarchical_block_poses[i - 1][count].size(); m++)
				{
					Hierarchical_block_poses[i - 1][count][m] = Hierarchical_block_poses[i][j][k] * Hierarchical_block_poses[i - 1][count][m];
				}
				count++;
			}
		}
	}
	int count = 0;
	for (int i = 0; i < Hierarchical_block_poses[0].size(); i++)
	{
		for (int j = 0; j < Hierarchical_block_poses[0][i].size(); j++)
		{
			global_poses[count]= Hierarchical_block_poses[0][i][j];
			count++;
		}
	}



}

void WHU::HL_MRF::eliminateClosestPoints(pcl::PointCloud<PointT>::Ptr& src,
	pcl::PointCloud<PointT>::Ptr& tgt, 
	Eigen::Matrix4f& trans, 
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr& fpfs,
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr& fpft)
{
	pcl::CorrespondencesPtr corr(new pcl::Correspondences);
	pcl::registration::CorrespondenceEstimation<PointT, PointT> es;
	es.setInputSource(src);
	es.setInputTarget(tgt);
	es.determineReciprocalCorrespondences(*corr, voxel_size_icp);

	int before = src->points.size();
	pcl::PointCloud<PointT>::Ptr after_earse(new pcl::PointCloud<PointT>());
	int count = 0;
	for (int i = 0; i < src->points.size(); i++)
	{
		if (i == (*corr)[count].index_query)
		{
			count++;
			continue;
		}
		after_earse->points.push_back(src->points[i]);
	}
	src->clear();
	src = after_earse;
	int after = src->points.size();
	cout << before - after << "points have been removed\n";
	count = 0;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr after_earse_feature(new pcl::PointCloud<pcl::FPFHSignature33>());
	if (fpfs.get() != nullptr)
	{
		for (int i = 0; i < fpfs->points.size(); i++)
		{
			if (i == (*corr)[count].index_query)
			{
				count++;
				continue;
			}
			after_earse_feature->points.push_back(fpfs->points[i]);
		}
		fpfs->clear();
		fpfs = after_earse_feature;
	}
	
}

double WHU::HL_MRF::getRMSE(pcl::PointCloud<PointT>::Ptr src, pcl::PointCloud<PointT>::Ptr tgt, Eigen::Matrix4f& trans,double max)
{
	double fitness_score = 0.0;

	// Transform the input dataset using the final transformation
	pcl::PointCloud<PointT> transformed;
	transformPointCloud(*src, transformed, trans);

	std::vector<int> nn_indices(1);
	std::vector<float> nn_dists(1);
	pcl::KdTreeFLANN<PointT>::Ptr tree_(new pcl::KdTreeFLANN<PointT>());
	tree_->setInputCloud(tgt);

	// For each point in the source dataset
	int nr = 0;
	for (std::size_t i = 0; i < transformed.points.size(); ++i)
	{
		// Find its nearest neighbor in the target
		tree_->nearestKSearch(transformed.points[i], 1, nn_indices, nn_dists);

		// Deal with occlusions (incomplete targets)
		if (nn_dists[0] <= max)
		{
			// Add to the fitness score
			fitness_score += nn_dists[0];
			nr++;
		}
	}

	if (nr > 0)
		return (fitness_score / nr);
	return (std::numeric_limits<double>::max());
}




void WHU::HL_MRF::performMultiviewRegistration()
{
	auto begin = std::chrono::steady_clock::now();
	//read PLY files
	if (this->PLYpath.empty())
	{
		std::cerr << "no input paths";
		return;
	}
	readPLYfiles();
	scans_left = files.size();
	global_poses.resize(files.size());
	scans_left_last = 0;

	/*Loop 1:internal block registration.
	* Loop 2:block-to-block registration.(default: only one "new block", higher level regisration code is not provided)
	* Termination criteria: 1.scans_left == 1, all scans are merged.
	*											  2.scans_left == scans_left, some scans can't merge.
	*/
	while (scans_left != 1 && scans_left_last != scans_left)
	{
		scans_left_last = scans_left;
		init();
		std::vector<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> block;
		for (int loop = 0; loop < part_loop; loop++)
		{
			blockPartition(loop);
			nr_pairs = pairs.size();

			//temporary varible for internal block merging
			std::vector <pcl::PointCloud <PointT>::Ptr> keypoint_clouds;
			std::vector <pcl::PointCloud<pcl::FPFHSignature33>::Ptr> keypoint_clouds_feature;
			std::vector <pcl::PointCloud <PointT>::Ptr> clouds;

			//temporary varible for global coarse registration
			std::vector<int> pairs_best_count(nr_pairs);
			std::vector <pcl::registration::MatchingCandidate, Eigen::aligned_allocator<pcl::registration::MatchingCandidate>> candidate_matches(nr_pairs);
			std::set<int> rejected_pairs_;

			//temporary varible for global fine registration
			int num_of_subtrees;
			std::vector<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> poses;

			transferVaribles(keypoint_clouds, keypoint_clouds_feature, clouds, loop);
			if (bigloop == 0)
			{
				preprocessing(keypoint_clouds, keypoint_clouds_feature, clouds, loop);
			}
			//perform pairwise registration
			coarseRgistration(keypoint_clouds, keypoint_clouds_feature, clouds, pairs_best_count, candidate_matches);
			//perform Global coarse and fine registration
			globalCoarseRegistration(rejected_pairs_, candidate_matches, pairs_best_count, nr_scans);
			globalFineRegistration(rejected_pairs_, candidate_matches, clouds, poses, num_of_subtrees);

			
			
			if (nr_scans >= 3)
			{
				//record the pose

				for (int f = 0; f < num_of_subtrees ; f++)
				{
					std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> temp0;
					for (int i = 0; i < poses[f].size(); i++)
					{
						if (poses[f][i].isZero())
							continue;
						temp0.push_back(poses[f][i]);


					}
					block.push_back(temp0);
				}

				//merge scans into block 
				for (int f = 0; f < num_of_subtrees; f++)
				{

					pcl::PointCloud<PointT>::Ptr part_sum(new pcl::PointCloud<PointT>);
					pcl::PointCloud<PointT>::Ptr part_sum_keypoint(new pcl::PointCloud<PointT>);
					pcl::PointCloud<pcl::FPFHSignature33>::Ptr part_sum_keypoint_feature(new pcl::PointCloud<pcl::FPFHSignature33>);
					for (int i = 0; i < poses[f].size(); i++)
					{
						if (poses[f][i].isZero())
							continue;

						pcl::transformPointCloud(*clouds[i], *clouds[i], poses[f][i]);
						pcl::transformPointCloud(*keypoint_clouds[i], *keypoint_clouds[i], poses[f][i]);
						// recording index for higher level icp matching
						*part_sum += *clouds[i];
						*part_sum_keypoint += *keypoint_clouds[i];
						*part_sum_keypoint_feature += *keypoint_clouds_feature[i];
						clouds[i]->clear();
						keypoint_clouds[i]->clear();
						keypoint_clouds_feature[i]->clear();

					}
					if (bigloop == 0)
					{
						part_clouds.push_back(part_sum);
						part_keypoint_clouds.push_back(part_sum_keypoint);
						part_keypoint_feature_clouds.push_back(part_sum_keypoint_feature);
					}
					else {
						part_clouds[all_count] = part_sum;
						part_keypoint_clouds[all_count] = part_sum_keypoint;
						part_keypoint_feature_clouds[all_count] = part_sum_keypoint_feature;
					}
					//std::stringstream ss;
					//ss << all_count;
					//pcl::io::savePCDFileBinary(output_dir + filename.str() + ss.str() + ".pcd", *part_clouds[all_count]);

					if (check_blcok)
					{
						pcl::visualization::PCLVisualizer vis;
						vis.addPointCloud<PointT>(part_clouds[all_count]);
						vis.spin();
					}
					all_count++;
				}
			}
			else {
				//if we have only two scans,just perfrom the pairwise registration and merge them
				if (nr_scans == 2)
				{
					if (pairs_best_count[0] > max_consensus_set)
					{


						cout << "apply  ICP" << endl;

						pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
						pcl::PointCloud<pcl::PointXYZ>::Ptr temp2(new pcl::PointCloud<pcl::PointXYZ>);
						copyPointCloud(*clouds[0], *temp);
						copyPointCloud(*clouds[1], *temp2);
						// pcl::transformPointCloud(*temp, *temp, candidate_matches[0][0].transformation);

						// CCCoreLib::ICPRegistrationTools::Parameters params;
						// params.finalOverlapRatio = approx_overlap;
						// params.maxThreadCount = 0;
						// ICP icp;
						// icp.setInputCloud(temp, temp2);
						// icp.setParameters(params);
						// icp.align();
						pcl::IterativeClosestPoint<PointT, PointT> icp;
						icp.setInputSource(temp);
						icp.setInputTarget(temp2);
						icp.setMaxCorrespondenceDistance(voxel_size_icp);
						icp.setUseReciprocalCorrespondences(true);
						icp.setMaximumIterations(10);
						icp.setEuclideanFitnessEpsilon(10e-4);
						icp.align(*temp, candidate_matches[0].transformation);
						candidate_matches[0].transformation = icp.getFinalTransformation();

						//record the pose
						
						std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> temp0;
						temp0.push_back(Eigen::Matrix4f::Identity());
						temp0.push_back(inverse(candidate_matches[0].transformation));
						block.push_back(temp0);


						pcl::PointCloud<PointT>::Ptr part_sum(new pcl::PointCloud<PointT>);
						pcl::PointCloud<PointT>::Ptr part_sum_keypoint(new pcl::PointCloud<PointT>);
						pcl::PointCloud<pcl::FPFHSignature33>::Ptr part_sum_keypoint_feature(new pcl::PointCloud<pcl::FPFHSignature33>);
						pcl::transformPointCloud(*clouds[1], *clouds[1], inverse(candidate_matches[0].transformation));
						pcl::transformPointCloud(*keypoint_clouds[1], *keypoint_clouds[1], inverse(candidate_matches[0].transformation));

						*part_sum += *clouds[0] + *clouds[1];
						*part_sum_keypoint += *keypoint_clouds[0] + *keypoint_clouds[1];
						*part_sum_keypoint_feature += *keypoint_clouds_feature[0] + *keypoint_clouds_feature[1];

						if (bigloop == 0)
						{
							part_clouds.push_back(part_sum);
							part_keypoint_clouds.push_back(part_sum_keypoint);
							part_keypoint_feature_clouds.push_back(part_sum_keypoint_feature);
						}
						else {
							part_clouds[all_count] = std::move(part_sum);
							part_keypoint_clouds[all_count] = std::move(part_sum_keypoint);
							part_keypoint_feature_clouds[all_count] = std::move(part_sum_keypoint_feature);
						}

						ofstream ofs3(output_dir + filename.str() + "_Mst" + ".txt");
						cout << "...saving results to files" << endl;

						ofs3 << "affine transformation: \n";
						ofs3 << Eigen::Matrix4f::Identity();
						ofs3 << "\n";

						ofs3 << "affine transformation: \n";
						ofs3 << inverse(candidate_matches[0].transformation);
						ofs3.close();

						//std::stringstream ss;
						//ss << all_count;
						//pcl::io::savePCDFileBinary(R"(D:\Programming\global_consistency\build\result\aligned_point_cloud)" + filename.str() + ss.str() + ".pcd", *part_clouds[all_count]);
						all_count++;

					}
					else {
						if (bigloop == 0)
						{
							part_clouds.push_back(clouds[0]);
							part_keypoint_clouds.push_back(keypoint_clouds[0]);
							part_keypoint_feature_clouds.push_back(keypoint_clouds_feature[0]);
							part_clouds.push_back(clouds[1]);
							part_keypoint_clouds.push_back(keypoint_clouds[1]);
							part_keypoint_feature_clouds.push_back(keypoint_clouds_feature[1]);
						}
						else {
							part_clouds[all_count] = std::move(clouds[0]);
							part_keypoint_clouds[all_count] = std::move(keypoint_clouds[0]);
							part_keypoint_feature_clouds[all_count] = std::move(keypoint_clouds_feature[0]);
							part_clouds[all_count + 1] = std::move(clouds[1]);
							part_keypoint_clouds[all_count + 1] = std::move(keypoint_clouds[1]);
							part_keypoint_feature_clouds[all_count + 1] = std::move(keypoint_clouds_feature[1]);
						}


						//record the pose

						std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> temp0;
						temp0.push_back(Eigen::Matrix4f::Identity());
						temp0.push_back(Eigen::Matrix4f::Identity());
						block.push_back(temp0);


						ofstream ofs3(output_dir + filename.str() + "_Mst0"  +".txt");
						cout << "...saving results to files" << endl;

						ofs3 << "affine transformation: \n";
						ofs3 << Eigen::Matrix4f::Identity();
						//ofs3 << "\n";
						ofs3.close();
						ofstream ofs4(output_dir + filename.str() + "_Mst1" +".txt");
						cout << "...saving results to files" << endl;

						ofs4 << "affine transformation: \n";
						ofs4 << Eigen::Matrix4f::Identity();
						//ofs4 << "\n";
						ofs4.close();
						//std::stringstream ss;
						//ss << all_count;
						//pcl::io::savePCDFileBinary(R"(D:\Programming\global_consistency\build\result\aligned_point_cloud)" + filename.str() + ss.str() + ".pcd", *part_clouds[all_count]);
						all_count++;
						//ss.clear();
						//ss << all_count;
						//pcl::io::savePCDFileBinary(R"(D:\Programming\global_consistency\build\result\aligned_point_cloud)" + filename.str() + ss.str() + ".pcd", *part_clouds[all_count]);
						all_count++;
					}
				}
				else {
				//if we have only one scans,just save them
					if (bigloop == 0)
					{
						part_clouds.push_back(clouds[0]);
						part_keypoint_clouds.push_back(keypoint_clouds[0]);
						part_keypoint_feature_clouds.push_back(keypoint_clouds_feature[0]);
					}
					else {
						part_clouds[all_count] = std::move(clouds[0]);
						part_keypoint_clouds[all_count] = std::move(keypoint_clouds[0]);
						part_keypoint_feature_clouds[all_count] = std::move(keypoint_clouds_feature[0]);
					}
					//record the pose

					std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> temp0;
					temp0.push_back(Eigen::Matrix4f::Identity());
					block.push_back(temp0);

					ofstream ofs3(output_dir + filename.str() + "_Mst0"  +".txt");
					cout << "...saving results to files" << endl;

					ofs3 << "affine transformation: \n";
					ofs3 << Eigen::Matrix4f::Identity();
					ofs3.close();
					//std::stringstream ss;
					//ss << all_count;
					//pcl::io::savePCDFileBinary(R"(D:\Programming\global_consistency\build\result\aligned_point_cloud)" + filename.str() + ss.str() + ".pcd", *part_clouds[all_count]);
					all_count++;
				}
			}
		}
		Hierarchical_block_poses.push_back(block);
		//check how many scans are left
		scans_left = 0;
		for (int i = 0; i < part_clouds.size(); i++)
		{
			if (part_clouds[i].get() == nullptr)
				continue;
			scans_left++;
		}
		part_clouds.resize(scans_left);
		part_keypoint_clouds.resize(scans_left);
		part_keypoint_feature_clouds.resize(scans_left);
		bigloop++;
	}

	auto end = std::chrono::steady_clock::now();
	std::chrono::duration<double> past = std::chrono::duration_cast<std::chrono::duration<double>>(end - begin);
	int min = past.count() / 60;
	double sec = past.count() - 60 * min;
	cout << "total cost time:" << min << "min" << sec << "sec\n";
	pcl::io::savePCDFile(output_dir + "/aligned_point_cloud.pcd", *part_clouds[0],true);
	try
	{
		solveGlobalPose();
	}
	catch(std::exception &e){
		e.what();
	}
	ofstream ofs3(output_dir +"/global_poses.txt");
	cout << "...saving final results to files" << endl;

	for (int i = 0; i < global_poses.size(); i++)
	{
		ofs3 << i+1<<": \n";
		ofs3 << global_poses[i]<<"\n";
	}
	ofs3.close();

}

// void WHU::HL_MRF::performShapeGrowingRegistration()
// {
// 	auto begin=std::chrono::steady_clock::now();
// 	//read PLY files
// 	if (this->PLYpath.empty())
// 	{
// 		std::cerr << "no input paths";
// 		return;
// 	}
// 	readPLYfiles();
// 	scans_left = files.size();
// 	scans_left_last = 0;

// 	init();
// 	std::vector<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> block;
// 	for (int loop = 0; loop < part_loop; loop++)
// 	{
// 		blockPartition(loop);
// 		nr_pairs = pairs.size();

// 		//temporary varible for internal block merging
// 		std::vector <pcl::PointCloud <PointT>::Ptr> keypoint_clouds;
// 		std::vector <pcl::PointCloud<pcl::FPFHSignature33>::Ptr> keypoint_clouds_feature;
// 		std::vector <pcl::PointCloud <PointT>::Ptr> clouds;

// 		//temporary varible for global coarse registration
// 		std::vector<int> pairs_best_count(nr_pairs);
// 		std::vector <std::vector <pcl::registration::MatchingCandidate, Eigen::aligned_allocator<pcl::registration::MatchingCandidate>>> candidate_matches(nr_pairs);
// 		std::vector <pcl::registration::MatchingCandidate, Eigen::aligned_allocator<pcl::registration::MatchingCandidate>> matches(nr_pairs);
// 		std::vector<int> rejected_pairs_;

// 		//temporary varible for global fine registration
// 		int num_of_subtrees;
// 		std::vector<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> poses;

// 		transferVaribles(keypoint_clouds, keypoint_clouds_feature, clouds, loop);
// 		if (bigloop == 0)
// 		{
// 			preprocessing(keypoint_clouds, keypoint_clouds_feature, clouds, loop);
// 		}
// 		//perform pairwise registration
// 		coarseRgistration(keypoint_clouds, keypoint_clouds_feature, clouds, pairs_best_count, candidate_matches);
// 		//perform Global coarse and fine registration
// 		globalCoarseRegistration(rejected_pairs_, candidate_matches, matches, pairs_best_count, nr_scans);
// 		globalFineRegistration(rejected_pairs_, matches, clouds, poses, num_of_subtrees);



// 		if (nr_scans >= 3)
// 		{
// 			//record the pose

// 			for (int f = 0; f < num_of_subtrees + 1; f++)
// 			{
// 				std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> temp0;
// 				for (int i = 0; i < poses[f].size(); i++)
// 				{
// 					if (poses[f][i].isZero())
// 						continue;
// 					temp0.push_back(poses[f][i]);


// 				}
// 				block.push_back(temp0);
// 			}

// 			//merge scans into block 
// 			for (int f = 0; f < num_of_subtrees + 1; f++)
// 			{

// 				pcl::PointCloud<PointT>::Ptr part_sum(new pcl::PointCloud<PointT>);
// 				pcl::PointCloud<PointT>::Ptr part_sum_keypoint(new pcl::PointCloud<PointT>);
// 				pcl::PointCloud<pcl::FPFHSignature33>::Ptr part_sum_keypoint_feature(new pcl::PointCloud<pcl::FPFHSignature33>);
// 				for (int i = 0; i < poses[f].size(); i++)
// 				{
// 					if (poses[f][i].isZero())
// 						continue;

// 					pcl::transformPointCloud(*clouds[i], *clouds[i], poses[f][i]);
// 					pcl::transformPointCloud(*keypoint_clouds[i], *keypoint_clouds[i], poses[f][i]);
// 					*part_sum += *clouds[i];
// 					*part_sum_keypoint += *keypoint_clouds[i];
// 					*part_sum_keypoint_feature += *keypoint_clouds_feature[i];
// 					clouds[i]->clear();
// 					keypoint_clouds[i]->clear();
// 					keypoint_clouds_feature[i]->clear();

// 				}
// 				if (bigloop == 0)
// 				{
// 					part_clouds.push_back(part_sum);
// 					part_keypoint_clouds.push_back(part_sum_keypoint);
// 					part_keypoint_feature_clouds.push_back(part_sum_keypoint_feature);
// 				}
// 				else {
// 					part_clouds[all_count] = part_sum;
// 					part_keypoint_clouds[all_count] = part_sum_keypoint;
// 					part_keypoint_feature_clouds[all_count] = part_sum_keypoint_feature;
// 				}
// 				//std::stringstream ss;
// 				//ss << all_count;
// 				//pcl::io::savePCDFileBinary(output_dir + filename.str() + ss.str() + ".pcd", *part_clouds[all_count]);

// 				if (check_blcok)
// 				{
// 					pcl::visualization::PCLVisualizer vis;
// 					vis.addPointCloud<PointT>(part_clouds[all_count]);
// 					vis.spin();
// 				}
// 				all_count++;
// 			}
// 		}
// 		else {
// 			//if we have only two scans,just perfrom the pairwise registration and merge them
// 			if (nr_scans == 2)
// 			{
// 				if (pairs_best_count[0] > max_consensus_set)
// 				{


// 					cout << "apply trimmed ICP" << endl;

// 					pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
// 					pcl::PointCloud<pcl::PointXYZ>::Ptr temp2(new pcl::PointCloud<pcl::PointXYZ>);
// 					copyPointCloud(*clouds[0], *temp);
// 					copyPointCloud(*clouds[1], *temp2);
// 					pcl::transformPointCloud(*temp, *temp, candidate_matches[0][0].transformation);

// 					CCCoreLib::ICPRegistrationTools::Parameters params;
// 					params.finalOverlapRatio = approx_overlap;
// 					params.maxThreadCount = 0;
// 					ICP icp;
// 					icp.setInputCloud(temp, temp2);
// 					icp.setParameters(params);
// 					icp.align();
// 					candidate_matches[0][0].transformation = icp.getFinalTransformation() * candidate_matches[0][0].transformation;

// 					//record the pose

// 					std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> temp0;
// 					temp0.push_back(Eigen::Matrix4f::Identity());
// 					temp0.push_back(inverse(candidate_matches[0][0].transformation));
// 					block.push_back(temp0);


// 					pcl::PointCloud<PointT>::Ptr part_sum(new pcl::PointCloud<PointT>);
// 					pcl::PointCloud<PointT>::Ptr part_sum_keypoint(new pcl::PointCloud<PointT>);
// 					pcl::PointCloud<pcl::FPFHSignature33>::Ptr part_sum_keypoint_feature(new pcl::PointCloud<pcl::FPFHSignature33>);
// 					pcl::transformPointCloud(*clouds[1], *clouds[1], inverse(candidate_matches[0][0].transformation));
// 					pcl::transformPointCloud(*keypoint_clouds[1], *keypoint_clouds[1], inverse(candidate_matches[0][0].transformation));
// 					*part_sum += *clouds[0] + *clouds[1];
// 					*part_sum_keypoint += *keypoint_clouds[0] + *keypoint_clouds[1];
// 					*part_sum_keypoint_feature += *keypoint_clouds_feature[0] + *keypoint_clouds_feature[1];

// 					if (bigloop == 0)
// 					{
// 						part_clouds.push_back(part_sum);
// 						part_keypoint_clouds.push_back(part_sum_keypoint);
// 						part_keypoint_feature_clouds.push_back(part_sum_keypoint_feature);
// 					}
// 					else {
// 						part_clouds[all_count] = std::move(part_sum);
// 						part_keypoint_clouds[all_count] = std::move(part_sum_keypoint);
// 						part_keypoint_feature_clouds[all_count] = std::move(part_sum_keypoint_feature);
// 					}

// 					ofstream ofs3(output_dir + filename.str() + "_Mst" + ".txt");
// 					cout << "...saving results to files" << endl;

// 					ofs3 << "affine transformation: \n";
// 					ofs3 << Eigen::Matrix4f::Identity();
// 					ofs3 << "\n";

// 					ofs3 << "affine transformation: \n";
// 					ofs3 << inverse(candidate_matches[0][0].transformation);
// 					ofs3.close();

// 					//std::stringstream ss;
// 					//ss << all_count;
// 					//pcl::io::savePCDFileBinary(R"(D:\Programming\global_consistency\build\result\aligned_point_cloud)" + filename.str() + ss.str() + ".pcd", *part_clouds[all_count]);
// 					all_count++;

// 				}
// 				else {
// 					if (bigloop == 0)
// 					{
// 						part_clouds.push_back(clouds[0]);
// 						part_keypoint_clouds.push_back(keypoint_clouds[0]);
// 						part_keypoint_feature_clouds.push_back(keypoint_clouds_feature[0]);
// 						part_clouds.push_back(clouds[1]);
// 						part_keypoint_clouds.push_back(keypoint_clouds[1]);
// 						part_keypoint_feature_clouds.push_back(keypoint_clouds_feature[1]);
// 					}
// 					else {
// 						part_clouds[all_count] = std::move(clouds[0]);
// 						part_keypoint_clouds[all_count] = std::move(keypoint_clouds[0]);
// 						part_keypoint_feature_clouds[all_count] = std::move(keypoint_clouds_feature[0]);
// 						part_clouds[all_count + 1] = std::move(clouds[1]);
// 						part_keypoint_clouds[all_count + 1] = std::move(keypoint_clouds[1]);
// 						part_keypoint_feature_clouds[all_count + 1] = std::move(keypoint_clouds_feature[1]);
// 					}


// 					//record the pose

// 					std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> temp0;
// 					temp0.push_back(Eigen::Matrix4f::Identity());
// 					temp0.push_back(Eigen::Matrix4f::Identity());
// 					block.push_back(temp0);


// 					ofstream ofs3(output_dir + filename.str() + "_Mst0" + ".txt");
// 					cout << "...saving results to files" << endl;

// 					ofs3 << "affine transformation: \n";
// 					ofs3 << Eigen::Matrix4f::Identity();
// 					//ofs3 << "\n";
// 					ofs3.close();
// 					ofstream ofs4(output_dir + filename.str() + "_Mst1" + ".txt");
// 					cout << "...saving results to files" << endl;

// 					ofs4 << "affine transformation: \n";
// 					ofs4 << Eigen::Matrix4f::Identity();
// 					//ofs4 << "\n";
// 					ofs4.close();
// 					//std::stringstream ss;
// 					//ss << all_count;
// 					//pcl::io::savePCDFileBinary(R"(D:\Programming\global_consistency\build\result\aligned_point_cloud)" + filename.str() + ss.str() + ".pcd", *part_clouds[all_count]);
// 					all_count++;
// 					//ss.clear();
// 					//ss << all_count;
// 					//pcl::io::savePCDFileBinary(R"(D:\Programming\global_consistency\build\result\aligned_point_cloud)" + filename.str() + ss.str() + ".pcd", *part_clouds[all_count]);
// 					all_count++;
// 				}
// 			}
// 			else {
// 				//if we have only one scans,just save them
// 				if (bigloop == 0)
// 				{
// 					part_clouds.push_back(clouds[0]);
// 					part_keypoint_clouds.push_back(keypoint_clouds[0]);
// 					part_keypoint_feature_clouds.push_back(keypoint_clouds_feature[0]);
// 				}
// 				else {
// 					part_clouds[all_count] = std::move(clouds[0]);
// 					part_keypoint_clouds[all_count] = std::move(keypoint_clouds[0]);
// 					part_keypoint_feature_clouds[all_count] = std::move(keypoint_clouds_feature[0]);
// 				}
// 				//record the pose

// 				std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> temp0;
// 				temp0.push_back(Eigen::Matrix4f::Identity());
// 				block.push_back(temp0);

// 				ofstream ofs3(output_dir + filename.str() + "_Mst0" + ".txt");
// 				cout << "...saving results to files" << endl;

// 				ofs3 << "affine transformation: \n";
// 				ofs3 << Eigen::Matrix4f::Identity();
// 				ofs3.close();
// 				//std::stringstream ss;
// 				//ss << all_count;
// 				//pcl::io::savePCDFileBinary(R"(D:\Programming\global_consistency\build\result\aligned_point_cloud)" + filename.str() + ss.str() + ".pcd", *part_clouds[all_count]);
// 				all_count++;
// 			}
// 		}
// 	}
// 	Hierarchical_block_poses.push_back(block);

// 	//shape-growing based registration
// 	for (int i = 0; i < part_clouds.size()-1; i++)
// 	{
// 		int n_optimal = 800; //optimal selection number
// 		cout << "Matching keypoints of " << 0 << " and " << i+1 << ".." << std::flush;
// 		int maxCorr = 5;
// 		pcl::CorrespondencesPtr corr(new pcl::Correspondences);
// 		std::vector<int> corrNOS, corrNOT;

// 		GenIn::correspondenceSearching(part_keypoint_feature_clouds[i+1], part_keypoint_feature_clouds[0], *corr, maxCorr, corrNOS, corrNOT);
// 		std::cout << "NO. corr = " << corr->size() << std::endl;

// 		pcl::registration::GRORInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, float> obor;
// 		pcl::PointCloud<pcl::PointXYZ>::Ptr pcs(new pcl::PointCloud<pcl::PointXYZ>);

// 		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_src(new pcl::PointCloud<pcl::PointXYZ>);
// 		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_tgt(new pcl::PointCloud<pcl::PointXYZ>);
// 		pcl::copyPointCloud<PointT, pcl::PointXYZ>(*part_keypoint_clouds[i+1], *temp_src);
// 		pcl::copyPointCloud<PointT, pcl::PointXYZ>(*part_keypoint_clouds[0], *temp_tgt);
// 		obor.setInputSource(temp_src);
// 		obor.setInputTarget(temp_tgt);
// 		obor.setResolution(voxel_size);
// 		obor.setOptimalSelectionNumber(n_optimal);
// 		obor.setNumberOfThreads(1);
// 		obor.setInputCorrespondences(corr);
// 		obor.setDelta(voxel_size);
// 		obor.align(*pcs);

		
		
// 		//icp
// 		cout << "apply trimmed ICP" << endl;
// 		Eigen::Matrix4f res;
// 		pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
// 		pcl::PointCloud<pcl::PointXYZ>::Ptr temp2(new pcl::PointCloud<pcl::PointXYZ>);
// 		copyPointCloud(*part_clouds[i+1], *temp);
// 		copyPointCloud(*part_clouds[0], *temp2);
// 		pcl::transformPointCloud(*temp, *temp, obor.getFinalTransformation());
// 		res = obor.getFinalTransformation();
// 		CCCoreLib::ICPRegistrationTools::Parameters params;
// 		params.finalOverlapRatio = approx_overlap;
// 		params.maxThreadCount = 0;
// 		ICP icp;
// 		icp.setInputCloud(temp, temp2);
// 		icp.setParameters(params);
// 		icp.align();
// 		res = icp.getFinalTransformation() * res;

// 		//combine into root scan

// 		pcl::transformPointCloud(*part_clouds[i + 1], *part_clouds[i + 1], res);
// 		pcl::transformPointCloud(*part_keypoint_clouds[i+1],* part_keypoint_clouds[i + 1], res);
// 		//eliminateClosestPoints(part_clouds[i + 1], part_clouds[0], res, pcl::PointCloud<pcl::FPFHSignature33>::Ptr(), pcl::PointCloud<pcl::FPFHSignature33>::Ptr());
// 		//eliminateClosestPoints(part_keypoint_clouds[i + 1], part_keypoint_clouds[0], res, part_keypoint_feature_clouds[i + 1], part_keypoint_feature_clouds[0]);
		
// 		*part_clouds[0] = *part_clouds[0] + *part_clouds[i + 1];
// 		*part_keypoint_clouds[0] =*part_keypoint_clouds[0] + *part_keypoint_clouds[i + 1];
// 	}
// 	auto end = std::chrono::steady_clock::now();
// 	std::chrono::duration<double> past = std::chrono::duration_cast<std::chrono::duration<double>>( end - begin);
// 	int min = past.count() / 60;
// 	double sec = past.count() - 60*min;
// 	cout << "total cost time:" << min << "min" << sec << "sec\n";
// 	pcl::io::savePCDFile(output_dir + "aligned_point_cloud.pcd", *part_clouds[0],true);
	
// }
