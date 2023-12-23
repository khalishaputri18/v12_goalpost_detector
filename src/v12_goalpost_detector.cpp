#include "v12_goalpost_detector/v12_goalpost_detector.h"

#define FROM_VIDEO 0

const float GoalpostDetector::MIN_CONTOUR_AREA = 100.0f;
const float GoalpostDetector::MIN_FIELD_CONTOUR_AREA = 1600.0f;

GoalpostDetector::GoalpostDetector()
    :nh_(ros::this_node::getName()),
      it_(this->nh_),
      it_subs_(it_.subscribe("image_in", 1, &GoalpostDetector::imageCallback, this)),
      it_pubs_(it_.advertise("image_out_goalpost", 100)),
      // cam_info_sub_(nh_.subscribe("camera_info_in", 100, &GoalpostDetector::cameraInfoCallback, this)),
      cam_info_pub_(nh_.advertise<sensor_msgs::CameraInfo>("camera_info_out", 100)),
      frame_mode_subs_(nh_.subscribe("frame_mode", 1, &GoalpostDetector::frameModeCallback, this)),
      save_param_subs_(nh_.subscribe("save_param", 1, &GoalpostDetector::saveParamCallback, this)),
      LUT_sub_(nh_.subscribe("LUT_data", 1, &GoalpostDetector::lutCallback, this)),
      it_bf_sub_(it_.subscribe("goalpost_ref", 1, &GoalpostDetector::goalpostRefCallback, this)),
      goalpost_pos_pub_(nh_.advertise<geometry_msgs::Point > ("goalpost_pos", 100)),
      update_params_pub_(nh_.advertise<std_msgs::Empty > ("update_params", 10)),
      it_inv_sg_pub_(it_.advertise("inv_segment_green", 10)),
      field_boundary_pub_(nh_.advertise<vision_utils::FieldBoundary > ("field_boundary", 10)),
      frame_mode_(0){

    nh_.param<std::string>("goalpost_config_path", goalpost_config_path,
                           ros::package::getPath("v12_goalpost_detector") + "/config/saved_config.yaml");

    param_cb_ = boost::bind(&GoalpostDetector::paramCallback, this, _1, _2);
    param_server_.setCallback(param_cb_);

    LUT_dir = ros::package::getPath("v12_goalpost_detector") + "/config/tabel_warna.xml";

    loadParam();
}
void GoalpostDetector::loadParam(){
    YAML::Node config_file;
    try{
        config_file = YAML::LoadFile(goalpost_config_path.c_str());
    }catch(const std::exception &e){
        ROS_ERROR("[v12_goalpost_detector] Unable to open config file: %s", e.what());
    }

    config_.canny1 = config_file["canny1"].as<int>();
    config_.canny2 = config_file["canny2"].as<int>();
    config_.intersections = config_file["intersections"].as<int>();
    config_.threshold_corner = config_file["threshold_corner"].as<int>();

    cv::FileStorage fs(LUT_dir.c_str(),cv::FileStorage::READ);
    fs["Tabel_Warna"] >> LUT_data;
    fs.release();

    goalpost_ref_ = cv::imread(ros::package::getPath("v12_goalpost_detector") + "/config/goalpost_ref.jpg");
    goalpost_ref_ = cvtMulti(goalpost_ref_);
    if(!goalpost_ref_.empty()){
        cv::calcHist(&goalpost_ref_, 1, hist_param_.channels, cv::Mat(), goalpost_ref_hist_, 2, hist_param_.hist_size, hist_param_.ranges);
        cv::normalize(goalpost_ref_hist_,goalpost_ref_hist_, .0, 1.0, cv::NORM_MINMAX);
    }
}

void GoalpostDetector::saveParam(){
    YAML::Emitter yaml_out;
    yaml_out << YAML::BeginMap;
    yaml_out << YAML::Key << "canny1" << YAML::Value << config_.canny1;
    yaml_out << YAML::Key << "canny2" << YAML::Value << config_.canny2;
    yaml_out << YAML::Key << "intersections" << YAML::Value << config_.intersections;
    yaml_out << YAML::Key << "threshold_corner" << YAML::Value << config_.threshold_corner;
    yaml_out << YAML::EndMap;
    std::ofstream file_out(goalpost_config_path.c_str());
    file_out << yaml_out.c_str();
    file_out.close();
    cv::FileStorage fs(LUT_dir.c_str(),cv::FileStorage::WRITE);
    fs << "Tabel_Warna" << LUT_data;
    fs.release();

    cv::imwrite(ros::package::getPath("v12_goalpost_detector") + "/config/goalpost_ref.jpg", goalpost_ref_);
}

void GoalpostDetector::frameModeCallback(const std_msgs::Int8::ConstPtr &_msg){
    frame_mode_ = _msg->data;
}

void GoalpostDetector::saveParamCallback(const std_msgs::Empty::ConstPtr &_msg){
    (void)_msg;
    saveParam();
}

void GoalpostDetector::goalpostRefCallback(const sensor_msgs::ImageConstPtr &_msg){
    cv_bf_ptr_sub_ = cv_bridge::toCvCopy(_msg,_msg->encoding);
    goalpost_ref_ = cv_bf_ptr_sub_->image;
    goalpost_ref_ = cvtMulti(goalpost_ref_);
    cv::calcHist(&goalpost_ref_, 1, hist_param_.channels, cv::Mat(), goalpost_ref_hist_, 2, hist_param_.hist_size, hist_param_.ranges);
    cv::normalize(goalpost_ref_hist_, goalpost_ref_hist_, .0, 1.0 , cv::NORM_MINMAX);
}

void GoalpostDetector::lutCallback(const vision_utils::LUTConstPtr &_msg){
    for(size_t i = 0; i < _msg->color.size(); i++){
        int h = (int)_msg->color[i].x;
        int s = (int)_msg->color[i].y;
        LUT_data.at<uchar>(h,s) = (int) _msg->color_class.data;
    }
}

void GoalpostDetector::imageCallback(const sensor_msgs::ImageConstPtr &_msg){

    try{
        img_encoding_ = Alfarobi::GRAY8Bit;
        if(_msg->encoding.compare(sensor_msgs::image_encodings::MONO8))
            img_encoding_ = Alfarobi::GRAY8Bit;
#if FROM_VIDEO == 0
        if(_msg->encoding.compare(sensor_msgs::image_encodings::BGR8))
            img_encoding_ = Alfarobi::BGR8Bit;
#else
        if(_msg->encoding.compare(sensor_msgs::image_encodings::RGB8))
            img_encoding_ = Alfarobi::BGR8Bit;
#endif
    }catch(cv_bridge::Exception &e){
        ROS_ERROR("[v12_goalpost_detector] cv bridge exception: %s",e.what());
        return;
    }

    cv_img_ptr_subs_ = cv_bridge::toCvCopy(_msg,_msg->encoding);
    this->stamp_ = _msg->header.stamp;
    this->frame_id_ = _msg->header.frame_id;
}

// void GoalpostDetector::cameraInfoCallback(const sensor_msgs::CameraInfo &_msg){
//     cam_info_msg_ = *_msg;
    
//     ROS_INFO("CHECK...");
// }

void GoalpostDetector::paramCallback(v12_goalpost_detector::GoalpostDetectorParamsConfig &_config, uint32_t level){
    (void)level;
    this->config_ = _config;
}

void GoalpostDetector::publishImage(){
    cv_img_pubs_.image = out_img_.clone();

    //Stamp
    cv_img_pubs_.header.seq++;
    cv_img_pubs_.header.stamp = this->stamp_;
    cv_img_pubs_.header.frame_id = this->frame_id_;

    //microsoft lifecam brightness setting only work when the camera is capturing
    //setting first to zero brightness after first 2 frame then set to desired value
    //3 April 2019
    if(cv_img_pubs_.header.seq == 2){
        std_msgs::Empty empty_msg;
        update_params_pub_.publish(empty_msg);
    }else if(cv_img_pubs_.header.seq == 4){
        std_msgs::Empty empty_msg;
        update_params_pub_.publish(empty_msg);
    }

    switch(img_encoding_){
        case Alfarobi::GRAY8Bit:cv_img_pubs_.encoding = sensor_msgs::image_encodings::MONO8;break;
        case Alfarobi::BGR8Bit:cv_img_pubs_.encoding = sensor_msgs::image_encodings::RGB8;break;
        default:cv_img_pubs_.encoding = sensor_msgs::image_encodings::RGB8;break;
    }

    it_pubs_.publish(cv_img_pubs_.toImageMsg());
    // cam_info_pub_.publish(cam_info_msg_);
}

cv::Mat& GoalpostDetector::setInputImage(){
    return in_img_;
}

void GoalpostDetector::setOutputImage(const cv::Mat &_out_img){
    out_img_ = _out_img.clone();
}

cv::Mat GoalpostDetector::segmentColor(cv::Mat &_segmented_green, cv::Mat &_inv_segmented_green, cv::Mat &_segmented_goalpost, cv::Mat &_segmented_background){

    cv::Mat blank = cv::Mat::zeros(Alfarobi::FRAME_HEIGHT, Alfarobi::FRAME_WIDTH, CV_8UC1);
    cv::Mat out_segment = cv::Mat::zeros(Alfarobi::FRAME_HEIGHT, Alfarobi::FRAME_WIDTH, CV_8UC3);

    cv::Mat segmented_green = blank.clone();
    cv::Mat segmented_yellow = blank.clone();
    cv::Mat segmented_black = blank.clone();
    cv::cvtColor(in_img_,in_hsv_,CV_BGR2HSV);

    int num_cols = Alfarobi::FRAME_WIDTH;
    int num_rows = Alfarobi::FRAME_HEIGHT;

    // auto LUT_ptr = LUT_data.data;
    for(int i = 0; i < num_rows; i++){
        cv::Vec3b* in_hsv_ptr = in_hsv_.ptr<cv::Vec3b>(i);
        cv::Vec3b* out_segment_ptr = out_segment.ptr<cv::Vec3b>(i);
        uchar* sg_ptr = segmented_green.ptr<uchar>(i);
        uchar* sy_ptr = segmented_yellow.ptr<uchar>(i);
        uchar* sb_ptr = segmented_black.ptr<uchar>(i); 
        for(int j = 0; j < num_cols; j++){
            uchar pres_class = LUT_data.at<uchar>(in_hsv_ptr[j][0], in_hsv_ptr[j][1]);
            if(pres_class == 1){
                sg_ptr[j] = 255;
                out_segment_ptr[j][0] = 0;
                out_segment_ptr[j][1] = 200;
                out_segment_ptr[j][2] = 0;                
            }else if(pres_class == 2){
                sy_ptr[j] = 255;
                out_segment_ptr[j][0] = 255;
                out_segment_ptr[j][1] = 255;
                out_segment_ptr[j][2] = 0;
            }else if(pres_class == 3){
                sb_ptr[j] = 255;
                out_segment_ptr[j][0] = 0;
                out_segment_ptr[j][1] = 0;
                out_segment_ptr[j][2] = 255;
            }
        }
    }

    cv::Mat inv_segmented_green;
    cv::bitwise_not(segmented_green,inv_segmented_green);

    _segmented_green = segmented_green.clone();
    _inv_segmented_green = inv_segmented_green.clone();
    _segmented_goalpost = segmented_yellow.clone();
    _segmented_background = segmented_black.clone();

    return out_segment;
}

cv::Mat GoalpostDetector::cvtMulti(const cv::Mat &_goalpost_ref){
    cv::Mat yuv;
    cv::cvtColor(_goalpost_ref,yuv,CV_BGR2YUV);
    return yuv.clone();
}

void GoalpostDetector::filterContourData(std::vector<cv::Mat> &divided_roi, cv::Point top_left_pt,
                       std::vector<Points > &selected_data, cv::Mat *debug_mat, int sub_mode = 0){
    int num_roi_cols = divided_roi[0].cols;
    int num_roi_rows = divided_roi[0].rows;
    bool horizon_scan = (float)num_roi_rows/(float)num_roi_cols < .75f;
    cv::Point map_origin[4];
    map_origin[0].x = top_left_pt.x;
    map_origin[0].y = top_left_pt.y;
    map_origin[1].x = (sub_mode == 2)?top_left_pt.x:top_left_pt.x + divided_roi[0].cols;
    map_origin[1].y = (sub_mode == 2)?top_left_pt.y + divided_roi[0].rows:top_left_pt.y;
    map_origin[2].x = top_left_pt.x;
    map_origin[2].y = top_left_pt.y + divided_roi[0].rows;
    map_origin[3].x = top_left_pt.x + divided_roi[0].cols;
    map_origin[3].y = top_left_pt.y + divided_roi[0].rows;
    for(size_t idx = 0;idx < divided_roi.size() ; idx++){

        int scan_mode=idx;

        switch(idx){
        case 0:scan_mode = (sub_mode == 1)?0:(sub_mode == 2)?2:horizon_scan?0:2;break;
        case 1:scan_mode = (sub_mode == 1)?1:(sub_mode == 2)?3:horizon_scan?1:2;break;
        case 2:scan_mode = horizon_scan?0:3;break;
        case 3:scan_mode = horizon_scan?1:3;break;
        }

        switch(scan_mode){
        case 0:{
            for(int i=0;i<num_roi_rows;i++){
                for(int j=0;j<num_roi_cols;j++){
                    if(divided_roi[idx].at<uchar>(i,j) == 255){
                        if(j==0)continue;
                        cv::Point selected_point;
                        selected_point.x = map_origin[idx].x + j;
                        selected_point.y = map_origin[idx].y + i;
                        selected_data[idx].push_back(selected_point);
                        debug_mat[idx].at<uchar>(i,j) = 255;
                        break;
                    }
                }
            }
        }break;
        case 1:{
            for(int i=0;i<num_roi_rows;i++){
                for(int j=num_roi_cols-1;j>=0;j--){
                    if(divided_roi[idx].at<uchar>(i,j) == 255){
                        if(j==num_roi_cols-1)continue;
                        cv::Point selected_point;
                        selected_point.x = map_origin[idx].x + j;
                        selected_point.y = map_origin[idx].y + i;
                        selected_data[idx].push_back(selected_point);
                        debug_mat[idx].at<uchar>(i,j) = 255;
                        break;
                    }
                }
            }
        }break;
        case 2:{
            for(int i=0;i<num_roi_cols;i++){
                for(int j=0;j<num_roi_rows;j++){
                    if(divided_roi[idx].at<uchar>(j,i) == 255){
                        if(j==0)continue;
                        cv::Point selected_point;
                        selected_point.x = map_origin[idx].x + i;
                        selected_point.y = map_origin[idx].y + j;
                        selected_data[idx].push_back(selected_point);
                        debug_mat[idx].at<uchar>(j,i) = 255;
                        break;
                    }
                }
            }
        }break;
        case 3:{
            for(int i=0;i<num_roi_cols;i++){
                for(int j=num_roi_rows-1;j>=0;j--){
                    if(divided_roi[idx].at<uchar>(j,i) == 255){
                        if(j==num_roi_rows-1)continue;
                        cv::Point selected_point;
                        selected_point.x = map_origin[idx].x + i;
                        selected_point.y = map_origin[idx].y + j;
                        selected_data[idx].push_back(selected_point);
                        debug_mat[idx].at<uchar>(j,i) = 255;
                        break;
                    }
                }
            }
        }break;

        }
    }
}


float GoalpostDetector::checkTargetHistogram(cv::Mat _target_roi){

    if(goalpost_ref_.empty()){
        ROS_ERROR("[v12_goalpost_detector] Goalpost reference not found !!!");
        return -1;
    }
    _target_roi = cvtMulti(_target_roi);
    cv::MatND target_hist;
    cv::calcHist(&_target_roi, 1, hist_param_.channels, cv::Mat(), target_hist, 2, hist_param_.hist_size, hist_param_.ranges);
    cv::normalize(target_hist,target_hist, .0, 1.0, cv::NORM_MINMAX);

    return cv::compareHist(goalpost_ref_hist_, target_hist, CV_COMP_KL_DIV);
}

void GoalpostDetector::publishLocalizationUtils(const cv::Mat &_inv_segmented_green,
                                                vision_utils::FieldBoundary _field_boundary){
    cv_inv_sg_pub_.image = _inv_segmented_green.clone();

    cv_inv_sg_pub_.header.seq++;
    _field_boundary.header.seq++;

    cv_inv_sg_pub_.header.stamp = this->stamp_;
    _field_boundary.header.stamp = this->stamp_;

    cv_inv_sg_pub_.header.frame_id = this->frame_id_;
    _field_boundary.header.frame_id = this->frame_id_;

    cv_inv_sg_pub_.encoding = sensor_msgs::image_encodings::MONO8;

    it_inv_sg_pub_.publish(cv_inv_sg_pub_.toImageMsg());
    field_boundary_pub_.publish(_field_boundary);
}

std::pair<cv::Mat, vision_utils::FieldBoundary > GoalpostDetector::getImageContours(const cv::Mat &_segmented_green){
    cv::Mat _field_contour = cv::Mat::zeros(_segmented_green.size(), CV_8UC1);
    vision_utils::FieldBoundary field_boundary;
    Points contour_points;
    std::vector<Points > contours;
    std::vector<cv::Vec4i > hierarchy;

    cv::findContours(_segmented_green, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    for(size_t i = 0; i < contours.size(); i++){
        if(cv::contourArea(contours[i]) > MIN_FIELD_CONTOUR_AREA){
            contour_points.insert(contour_points.end(), contours[i].begin(), contours[i].end());
        }
    }

    if(contour_points.size()){
        std::vector<Points > contour(1);
        cv::convexHull(contour_points,contour[0]);
        cv::Rect field_bound = cv::boundingRect(contour[0]);
        drawContours(_field_contour, contour, 0, cv::Scalar(255), CV_FILLED);
        //[HW] Scan from dual direction
        for(int i=field_bound.tl().x;
            i<field_bound.br().x;i++){
            geometry_msgs::Vector3 temp;
            temp.x = i;
            temp.y = -1;
            temp.z = field_bound.br().y-1;
            for(int j=field_bound.tl().y;
                j<field_bound.br().y;j++){
                if(_field_contour.at<uchar >(j,i) > 0 &&
                        temp.y==-1){
                    temp.y = j;
                }else if(_field_contour.at<uchar >(j,i) == 0 &&
                         temp.y!=-1){
                    temp.z = j-1;
                    break;
                }
            }
            field_boundary.bound1.push_back(temp);
        }

        for(int i=field_bound.tl().y;
            i<field_bound.br().y;i++){
            geometry_msgs::Vector3 temp;
            temp.x = i;
            temp.y = -1;
            temp.z = field_bound.br().x-1;
            for(int j=field_bound.tl().x;
                j<field_bound.br().x;j++){
                if(_field_contour.at<uchar >(i,j) > 0 &&
                        temp.y==-1){
                    temp.y = j;
                }else if(_field_contour.at<uchar >(i,j) == 0 &&
                         temp.y!=-1){
                    temp.z = j-1;
                    break;
                }
            }
            field_boundary.bound2.push_back(temp);
        }
    }

    std::pair<cv::Mat, vision_utils::FieldBoundary > result;
    result.first = _field_contour;
    result.second = field_boundary;
    return result;
}

bool isEqual(const cv::Vec4i& _l1, const cv::Vec4i& _l2)
{
    cv::Vec4i l1(_l1), l2(_l2);

    float length1 = sqrtf((l1[2] - l1[0])*(l1[2] - l1[0]) + (l1[3] - l1[1])*(l1[3] - l1[1]));
    float length2 = sqrtf((l2[2] - l2[0])*(l2[2] - l2[0]) + (l2[3] - l2[1])*(l2[3] - l2[1]));

    float product = (l1[2] - l1[0])*(l2[2] - l2[0]) + (l1[3] - l1[1])*(l2[3] - l2[1]);

    if (fabs(product / (length1 * length2)) < cos(CV_PI / 30))
        return false;

    float mx1 = (l1[0] + l1[2]) * 0.5f;
    float mx2 = (l2[0] + l2[2]) * 0.5f;

    float my1 = (l1[1] + l1[3]) * 0.5f;
    float my2 = (l2[1] + l2[3]) * 0.5f;
    float dist = sqrtf((mx1 - mx2)*(mx1 - mx2) + (my1 - my2)*(my1 - my2));

    if (dist > std::max(length1, length2) * 0.5f)
        return false;

    return true;
}

cv::Vec2d linearParameters(cv::Vec4i line){
    cv::Mat a = (cv::Mat_<double>(2, 2) <<
                line[0], 1,
                line[2], 1);
    cv::Mat y = (cv::Mat_<double>(2, 1) <<
                line[1],
                line[3]);
    cv::Vec2d mc; solve(a, y, mc);
    return mc;
}

cv::Vec4i extendedLine(cv::Vec4i line, double d){
    // oriented left-t-right
    cv::Vec4d _line = line[2] - line[0] < 0 ? cv::Vec4d(line[2], line[3], line[0], line[1]) : cv::Vec4d(line[0], line[1], line[2], line[3]);
    double m = linearParameters(_line)[0];
    // solution of pythagorean theorem and m = yd/xd
    double xd = sqrt(d * d / (m * m + 1));
    double yd = xd * m;
    return cv::Vec4d(_line[0] - xd, _line[1] - yd , _line[2] + xd, _line[3] + yd);
}

std::vector<cv::Point2i> boundingRectangleContour(cv::Vec4i line, float d){
    // finds coordinates of perpendicular lines with length d in both line points
    // https://h.stackexchange.com/a/2043065/183923

    cv::Vec2f mc = linearParameters(line);
    float m = mc[0];
    float factor = sqrtf(
        (d * d) / (1 + (1 / (m * m)))
    );

    float x3, y3, x4, y4, x5, y5, x6, y6;
    // special case(vertical perpendicular line) when -1/m -> -infinity
    if(m == 0){
        x3 = line[0]; y3 = line[1] + d;
        x4 = line[0]; y4 = line[1] - d;
        x5 = line[2]; y5 = line[3] + d;
        x6 = line[2]; y6 = line[3] - d;
    } else {
        // slope of perpendicular lines
        float m_per = - 1/m;

        // y1 = m_per * x1 + c_per
        float c_per1 = line[1] - m_per * line[0];
        float c_per2 = line[3] - m_per * line[2];

        // coordinates of perpendicular lines
        x3 = line[0] + factor; y3 = m_per * x3 + c_per1;
        x4 = line[0] - factor; y4 = m_per * x4 + c_per1;
        x5 = line[2] + factor; y5 = m_per * x5 + c_per2;
        x6 = line[2] - factor; y6 = m_per * x6 + c_per2;
    }

    return std::vector<cv::Point2i> {
            cv::Point2i(x3, y3),
            cv::Point2i(x4, y4),
            cv::Point2i(x6, y6),
            cv::Point2i(x5, y5)
        };
    }

bool extendedBoundingRectangleLineEquivalence(const cv::Vec4i& _l1, const cv::Vec4i& _l2, float extensionLengthFraction, float maxAngleDiff, float boundingRectangleThickness){

    cv::Vec4i l1(_l1), l2(_l2);
    // extend lines by percentage of line width
    float len1 = sqrtf((l1[2] - l1[0])*(l1[2] - l1[0]) + (l1[3] - l1[1])*(l1[3] - l1[1]));
    float len2 = sqrtf((l2[2] - l2[0])*(l2[2] - l2[0]) + (l2[3] - l2[1])*(l2[3] - l2[1]));
    cv::Vec4i el1 = extendedLine(l1, len1 * extensionLengthFraction);
    cv::Vec4i el2 = extendedLine(l2, len2 * extensionLengthFraction);

    // reject the lines that have wide difference in angles
    float a1 = atan(linearParameters(el1)[0]);
    float a2 = atan(linearParameters(el2)[0]);
    if(fabs(a1 - a2) > maxAngleDiff * M_PI / 180.0){
        return false;
    }

    // calculate window around extended line
    // at least one point needs to inside extended bounding rectangle of other line,
    std::vector<cv::Point2i> lineBoundingContour = boundingRectangleContour(el1, boundingRectangleThickness/2);
    return
        cv::pointPolygonTest(lineBoundingContour, cv::Point(el2[0], el2[1]), false) == 1 ||
        cv::pointPolygonTest(lineBoundingContour, cv::Point(el2[2], el2[3]), false) == 1;
}



void GoalpostDetector::process(){
    if(cv_img_ptr_subs_ == nullptr)return;
    static geometry_msgs::Point last_goalpost_pos_;
    setInputImage() = cv_img_ptr_subs_->image;

    cv::Mat output_view = in_img_.clone();
    cv::Mat segmented_green, inv_segmented_green, segmented_yellow, segmented_background;
    cv::Mat thresh_image = segmentColor(segmented_green, inv_segmented_green, segmented_yellow, segmented_background);
   
    cv::Mat field_contour;
    std::pair<cv::Mat, vision_utils::FieldBoundary > field_prop = getImageContours(segmented_green);
    field_contour = field_prop.first;
    publishLocalizationUtils(inv_segmented_green,field_prop.second);
   
    cv::Mat background_contour;
    std::pair<cv::Mat, vision_utils::FieldBoundary > background_prop = getImageContours(segmented_background);
    background_contour = background_prop.first;
   
    cv::Mat goalpost_in_background;
    cv::bitwise_and(segmented_yellow,background_contour,goalpost_in_background);

    cv::Mat ver = goalpost_in_background.clone();
    int hor_size = ver.cols/80;
    int ver_size = ver.rows/40;
    cv::Mat ver_struktur = cv::getStructuringElement(cv::MorphShapes::MORPH_RECT, cv::Size(hor_size, ver_size));
    cv::dilate(ver, ver, ver_struktur, cv::Point(-1,-1));
    cv::erode(ver, ver, ver_struktur, cv::Point(-1,-1));

    cv::Mat skel(ver.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat temp(ver.size(), CV_8UC1);
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3,3));

    bool done;
    do
    {
        cv::morphologyEx(ver, temp, cv::MORPH_OPEN, element);
        cv::bitwise_not(temp, temp);
        cv::bitwise_and(ver, temp, temp);
        cv::bitwise_or(skel, temp, skel);
        cv::erode(ver, ver, element);
    
        double max;
        cv::minMaxLoc(ver, 0, &max);
        done = (max == 0);
    } while (!done);

    std::vector<cv::Vec4i> linesP;

    cv::Mat garis = cv::Mat::zeros(output_view.rows, output_view.cols,CV_8UC1);
    cv::Mat dot = cv::Mat::zeros(output_view.rows, output_view.cols,CV_8UC1);
    cv::HoughLinesP(skel, linesP, 1, CV_PI/180, 50, 50, 10);

    for( size_t i = 0; i < linesP.size(); i++ )
    {
        cv::Vec4i l = linesP[i];
        line( garis, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255,0,0), 1, cv::LINE_AA);

        std::vector<cv::Point> pts;
        for(unsigned int z = 0; z < linesP.size(); z++)
        {
            pts.push_back(cv::Point(linesP[z][0], linesP[z][1]));
            pts.push_back(cv::Point(linesP[z][2], linesP[z][3]));
        }
        cv::Rect box = boundingRect(pts);
        cv::rectangle(output_view, box.tl(), box.br(), cv::Scalar(0,255,0), 2);
        cv::Point center_rect = (box.br() + box.tl())/2;
        cv::circle(output_view, center_rect, 5, cv::Scalar(0,0,255));

        if(!linesP.size()){
            goalpost_pos_.x = -1;
            goalpost_pos_.y = -1;
            goalpost_pos_.z = 0;
            goalpost_pos_pub_.publish(goalpost_pos_);
        }else {
            goalpost_pos_.x = center_rect.x;
            goalpost_pos_.y = center_rect.y;
            goalpost_pos_.z = 0;
            goalpost_pos_pub_.publish(goalpost_pos_);
        }






    }
    #if DEBUG == 1
    ROS_WARN("[Settings] Canny 1: %i", config_.canny1);
    ROS_WARN("[Settings] Canny 2: %i", config_.canny2);
    ROS_WARN("[Settings] Intersections: %i", config_.intersections);
    ROS_WARN("[Settings] Threshold: %i", config_.threshold_corner);    
#endif

    //For purpose GUI only
    std::cout << frame_mode_ << std::endl;
    switch(frame_mode_){
    case 1:setOutputImage(in_hsv_);break;
    case 2:setOutputImage(thresh_image);break;
    case 3:setOutputImage(output_view);break;
    default:setOutputImage(in_img_);break;
    }
    publishImage();
    }