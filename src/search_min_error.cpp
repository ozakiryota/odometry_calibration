#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class SearchMinError{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*objects*/
		double ini_r;
		double ini_l;
		double ini_t;
		double fin_x;
		double fin_y;
		double fin_theta;
		double rl_onestep;
		double t_onestep;
		int resolution;
		/*file*/
		// FILE *fp;
	public:
		SearchMinError();
		void GetFinal(void);
		void Search(void);
		double ComputationError(double x, double y);
		void Integration(double r, double l, double t, double& x, double& y, double& theta);
};

SearchMinError::SearchMinError()
	: nhPrivate("~")
{
	nhPrivate.param("ini_r", ini_r, 0.125);
	nhPrivate.param("ini_l", ini_l, 0.125);
	nhPrivate.param("ini_t", ini_t, 0.45);
	nhPrivate.param("rl_onestep", rl_onestep, 0.0001);
	nhPrivate.param("t_onestep", t_onestep, 0.0001);
	nhPrivate.param("resolution", resolution, 100);

	GetFinal();
}

void SearchMinError::GetFinal(void)
{
	FILE *fp;
	if((fp = fopen("/home/amsl/Desktop/data.csv", "r")) == NULL){
		printf("file open error!!\n");
		exit(EXIT_FAILURE);
	}

	while(fscanf(fp, "%lf,%lf\n", &fin_x, &fin_y) != EOF){
		if(std::isnan(fin_x) || std::isnan(fin_y))	break;
	}
	if(fscanf(fp, "%lf,%lf,%lf\n", &fin_x, &fin_y, &fin_theta) == EOF){
		std::cout << "fscanf error" << std::endl;
		exit(1);
	}

	std::cout << "fin_x = " << fin_x << std::endl;
	std::cout << "fin_y = " << fin_y << std::endl;
	std::cout << "fin_theta/M_PI*180.0 = " << fin_theta/M_PI*180.0 << std::endl;

	fclose(fp);
}
#include<omp.h>
void SearchMinError::Search(void)
{
	double x, y, theta;
	Integration(ini_r, ini_l, ini_t, x, y, theta);
	double opt_r = ini_r;
	double opt_l = ini_l;
	double opt_t = ini_t;
	double min_error = ComputationError(x, y);


	std::cout << "rl: " << -resolution/2*rl_onestep << " ~ +" << resolution/2*rl_onestep << std::endl;
	std::cout << "t: " << -resolution/2*t_onestep << " ~ +" << resolution/2*t_onestep << std::endl;
	std::cout << "resolution = " << resolution << std::endl;
	std::cout << "Searching..." << std::endl;
	ros::Time t_start = ros::Time::now();

	#pragma omp parallel for
	for(int i=-resolution/2;i<resolution/2;i++){
		for(int j=-resolution/2;j<resolution/2;j++){
			for(int k=-resolution/2;k<resolution/2;k++){
				double r = ini_r + i*rl_onestep;
				double l = ini_l + j*rl_onestep;
				double t = ini_t + k*t_onestep;
				Integration(r, l, t, x, y, theta);
				double error = ComputationError(x, y);
				if(error<min_error){
					min_error = error;
					opt_r = r;
					opt_l = l;
					opt_t = t;
				}
			}
		}
		/* std::cout << "Progress: " << (i + resolution/2)/(double)resolution*100 << " [%]" << std::endl; */
	}

	ros::Time t_end = ros::Time::now();
	std::cout << "Searching time: " << (t_end - t_start).toSec() << std::endl;

	std::cout << "ini_r = " << ini_r << std::endl;
	std::cout << "ini_l = " << ini_l << std::endl;
	std::cout << "ini_t = " << ini_t << std::endl;
	std::cout << "opt_r = " << opt_r << std::endl;
	std::cout << "opt_l = " << opt_l << std::endl;
	std::cout << "opt_t = " << opt_t << std::endl;
}

void SearchMinError::Integration(double r, double l, double t, double& x, double& y, double& theta)
{
	FILE *fp;
	if((fp = fopen("/home/amsl/Desktop/data.csv", "r")) == NULL){
		printf("file open error!!\n");
		exit(EXIT_FAILURE);
	}

	x = 0.0;
	y = 0.0;
	theta = 0.0;

	double wr, wl;
	while(fscanf(fp, "%lf,%lf\n", &wr, &wl) != EOF){
		/* std::cout << "wr, wl = " << wr << ", " << wl << std::endl; */
		if(std::isnan(wr) || std::isnan(wl))	break;

		double v = (r*wr + l*wl)/2.0;
		double w = (r*wr - l*wl)/t;

		x += v*cos(theta);
		y += v*sin(theta);
		theta -= w;
	}

	/* std::cout << "x = " << x << std::endl; */
	/* std::cout << "y = " << y << std::endl; */
	/* std::cout << "theta/M_PI*180.0 = " << theta/M_PI*180.0 << std::endl; */

	fclose(fp);
}

double SearchMinError::ComputationError(double x, double y)
{
	double del_x = fin_x - x;
	double del_y = fin_y - y;
	return sqrt(del_x*del_x + del_y*del_y);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "search_min_error");

	SearchMinError search_min_error;

	search_min_error.Search();
}
