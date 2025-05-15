
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// for hash function
#define HASH_P 116101
#define MAX_N 10000000000
#define MAX_FRAME_N 20000

struct GrpStaic
{
	long lGrpID;    // Group ID
	long lEleNum;   // Group element number
	long lStaID;	// Start element ID
	long lEndID;    // End element ID
};

struct ProfLabel
{
	long lXPrior;       //X Prior
	long lXPstor;		//X Postorior
	long lYPrior;       //Y Prior
	long lYPstor;		//Y Postorior
	long lLabel;        //Group ID
};	

// parameters for preprocessing, std, and place recognition
typedef struct ConfigSetting {
	
	// project to the image
	int imgRow = 3600;
	int imgCol = 1800;

  double range = 100;
  int grid_num = 100;
	
	// for segment the trunk
	int isRow_Or_Column = 2;
	double dDif_Th = 0.05;
	int nSch_Stp = 10;
	int lGrp_Ele_Min = 15;

	// for tri descriptor
	int descriptor_near_num = 10;
	double descriptor_min_len = 2;
	double descriptor_max_len = 50;
	double side_resolution = 0.1;
	
	// for place recognition
	int candidate_num = 50;
	double rough_dis_threshold = 0.1;
	double vertex_diff_threshold = 0;
	double icp_threshold = 0.5;
	double dist_candi_frames_verify = 2;
	double normal_geo_verify = 1;
	double dis_geo_verify = 0.3;
	int use_matched_num = 4000;
	
	// for CSF, ground filter
	bool bSloopSmooth = false;
	double cloth_resolution = 0.5;
	int rigidness = 3;
	double time_step = 0.65;
	double class_threshold = 0.5;
	int interations = 500;  

	// for FEC Cluster
	int min_component_size = 50;
	double tolorance = 0.2;
	int max_n = 50;
	double merge_dist = 0.1;
	
	// for ground Cluster
	double gnd_grid_size = 0.5;
	double gnd_points_num = 5;
	double gnd_points_dist = 0.5;
	
	// to disguish the cluster type
	double linearityThres = 0.95;
  double upThres = 0.3;
	double scateringThres = 0.05;

  double clusterHeight = 2.5;
  int centerSelection = 0;

	// for plane selection
	int pointsNumThres = 10;
	double planarityThres = 0.7;

} ConfigSetting;

// for point to image
struct POINT_D {
  pcl::PointXYZ p;
  double d = -1;
};

// structure for Cluster
typedef struct Cluster {
  pcl::PointXYZINormal p_center_;
  pcl::PointCloud<pcl::PointXYZ> points_;
  Eigen::Vector3d center_;
  Eigen::Vector3d normal_;
  Eigen::Matrix3d covariance_;
  Eigen::Vector3d eig_value_;
  double minZ;
  double maxZ;
  double root;
  double linearity_ = 0;
  double planarity_ = 0;
  double scatering_ = 0;
  bool is_plane_ = false;
  bool is_line_ = false;
} Cluster;

// Structure for Multi-level Descriptor
typedef struct TriDesc {
  // the side lengths of STDesc, arranged from short to long
  Eigen::Vector3d side_length_;

  // projection angle between vertices
  Eigen::Vector3d angle_;

  Eigen::Vector3d center_;
  unsigned int frame_id_;

  // three vertexs
  Eigen::Vector3d vertex_A_;
  Eigen::Vector3d vertex_B_;
  Eigen::Vector3d vertex_C_;

  // some other inform attached to each vertex,e.g., intensity
  Eigen::Vector3d vertex_attached_;
} TriDesc;

typedef struct FrameInfo{
  std::vector<TriDesc> desc_;
  unsigned int frame_id_;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr currCenter;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr currCenterFix;
  pcl::PointCloud<pcl::PointXYZ>::Ptr currPoints;
}FrameInfo;

// std descriptor match lists
typedef struct TriMatchList {
  std::vector<std::pair<TriDesc, TriDesc>> match_list_;
  std::pair<int, int> match_id_;
  double mean_dis_;
} TriMatchList;

// candidate information
typedef struct CandidateInfo {
  int currFrameID;
  std::vector<std::pair<int, double>> candidateIDScore;
  std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> relativePose;
  std::vector<std::vector<std::pair<TriDesc, TriDesc>>> triMatch;
} CandidateInfo;

// pose information of TLS stations
typedef struct TLSPos {
  int ID;
  Eigen::Vector3d t;
  Eigen::Matrix3d R;
  bool isValued = false;
}TLSPos;

typedef struct PosError {
  int ID;
  std::pair<Eigen::Vector3d, Eigen::Vector3d> error;
}PosError;

// location of STD, and a operator to define whether is equal or not
class TriDesc_LOC {
public:
  int64_t x, y, z, a, b, c;

  TriDesc_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0, int64_t va = 0,
             int64_t vb = 0, int64_t vc = 0)
      : x(vx), y(vy), z(vz), a(va), b(vb), c(vc) {}

  bool operator==(const TriDesc_LOC &other) const {
    // use three attributes
    return (x == other.x && y == other.y && z == other.z);
    // use six attributes
    // return (x == other.x && y == other.y && z == other.z && a == other.a &&
    //         b == other.b && c == other.c);
  }
};

// hash mapping for TriDesc_LOC, input TriDesc_LOC, output int64
template <> struct std::hash<TriDesc_LOC> {
  int64_t operator()(const TriDesc_LOC &s) const {
    using std::hash;
    using std::size_t;
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
  }
};

// location of uniform voxel
class UNI_VOXEL_LOC {
public:
    int64_t x, y, z;

    UNI_VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
        : x(vx), y(vy), z(vz) {}

    bool operator==(const UNI_VOXEL_LOC &other) const {
        return (x == other.x && y == other.y && z == other.z);
    }
};

// hash mapping for VOXEL_LOC, input VOXEL_LOC, output int64
template <> struct std::hash<UNI_VOXEL_LOC> {
  int64_t operator()(const UNI_VOXEL_LOC &s) const {
    using std::hash;
    using std::size_t;
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
  }
};

// for down sample function
struct M_POINT {
  float xyz[3];
  int count = 0;
};