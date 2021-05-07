#include "geometry/util.h"
namespace geometry
{
namespace util {

int resample_polyline(const RowMatrixXd& polyline, double step, RowMatrixXd& ret)
		{
			  if(polyline.rows() < 2)
			  {
				  ret=polyline.eval();
				  return 0;
			  }


			  geometry::EigenPolyline new_polyline;
			  new_polyline.push_back(polyline.middleRows(0,1).transpose().eval());
			  double current_position = step;
			  double current_length = (polyline.middleRows(0,1)-polyline.middleRows(1,1)).norm();
			  int current_idx = 0;
			  while(current_idx < (polyline.rows() - 1))
			  {
				  if(current_position >= current_length)
				  {
					  current_position = current_position - current_length;
					  current_idx += 1;
					  if(current_idx > polyline.rows() - 2)
						  break;
					  current_length = (polyline.middleRows(current_idx + 1,1)-polyline.middleRows(current_idx,1)).norm();
				  }
				  else
				  {
					  double rel = current_position / current_length;
					  new_polyline.push_back(((1 - rel) * polyline.middleRows(current_idx,1) +
										  rel * polyline.middleRows(current_idx+1,1)).eval().transpose().eval());
					  current_position += step;
				  }
			  }
			  new_polyline.push_back(polyline.middleRows(polyline.rows()-1,1).transpose().eval());

			  ret=RowMatrixXd (new_polyline.size(),polyline.cols());
			  for(int cc1=0; cc1< new_polyline.size(); cc1++)
			  {
				  ret.middleRows(cc1,1)=new_polyline[cc1].transpose().eval();
			  }

			  return 0;
		}


		int chaikins_corner_cutting(const RowMatrixXd& polyline, int refinements, RowMatrixXd& ret)
		{

			RowMatrixXd el2=polyline.eval();
			for(int cc1=0; cc1<refinements; cc1++)
			{
				RowMatrixXd el (2*el2.rows(), el2.cols());

				int j = 0;

				for (int i = 0; i < el2.rows(); ++i) {

					const int k = 2;
					el.middleRows(j, 2) = el2.row(i).colwise().replicate(2);
					j += k;
				}

				RowMatrixXd R=RowMatrixXd::Zero(el.rows(), el.cols());

				R.middleRows(0,1)=el.middleRows(0,1);

				for(int cc1=2; cc1<R.rows(); cc1+=2)
				{
					R.middleRows(cc1,1)=el.middleRows(cc1-1,1);
					R.middleRows(cc1-1,1)=el.middleRows(cc1,1);

				}
				R.middleRows(R.rows()-1,1)=el.middleRows(el.rows()-1,1);

				el2=(el*0.75+R*0.25).eval();
			}

			ret= el2;
			return 0;
		}
}
}
