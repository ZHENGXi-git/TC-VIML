#include "estimator.h"

int ALL_FRAME = 0;

double all_lines = 0;
int useful_frame = 0;
double average_lines = 0;

Eigen::Matrix3d rpy2R(const Eigen::Vector3d &rpy)
{
    double r = rpy(0) / 180.0 * M_PI;
    double p = rpy(1) / 180.0 * M_PI;
    double y = rpy(2) / 180.0 * M_PI;

    Eigen::Matrix3d Rx;
    Rx << 1., 0., 0.,
            0., cos(r), -sin(r),
            0., sin(r), cos(r);

    Eigen::Matrix3d Ry;
    Ry << cos(p), 0., -sin(p),
            0., 1., 0.,
            sin(p), 0., cos(p);

    Eigen::Matrix3d Rz;
    Rz << cos(y), -sin(y), 0,
            sin(y), cos(y), 0,
            0, 0, 1;

    return Rz * Ry * Rx;
}

Estimator::Estimator(): f_manager{Rs}
{
    ROS_INFO("init begins");
    clearState();
}

void Estimator::setParameter()
{

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
    }
    f_manager.setRic(ric);
    ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTdFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();

    td = TD;
}

void Estimator::setParameters(const string &calib_file, vector<Vector6d> &_lines3d)
{

    lines3d_map = _lines3d;

    std::cout << "the line number of the lines3d_map = " << lines3d_map.size() << endl;

    cv::FileStorage fsSettings(calib_file, cv::FileStorage::READ);

    double m_fx = fsSettings["fx"];
    double m_fy = fsSettings["fy"];
    double m_cx = fsSettings["cx"];
    double m_cy = fsSettings["cy"];
    cv_KMatrix = (cv::Mat_<double>(3, 3) << m_fx, 0, m_cx, 0, m_fy, m_cy, 0, 0, 1);
    cv::cv2eigen(cv_KMatrix, K);

    double m_k1 = fsSettings["k1"];
    double m_k2 = fsSettings["k2"];
    double m_p1 = fsSettings["p1"];
    double m_p2 = fsSettings["p2"];
    cv_dist = (cv::Mat_<double>(1, 4) << m_k1, m_k2, m_p1, m_p2);
    width = static_cast<int>(fsSettings["width"]);
    height = static_cast<int>(fsSettings["height"]);
 //   undisKeyLine = fsSettings["undisKeyLine"];

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
    }
    f_manager.setRic(ric);
    ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTdFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();

    td = TD;
    td = 0;

    cv::Mat cv_R, cv_T;
    fsSettings["initialRotation"] >> cv_R;
    fsSettings["initialTranslation"] >> cv_T;
    Eigen::Matrix3d eigen_R;
    Eigen::Vector3d eigen_T;
    cv::cv2eigen(cv_R, eigen_R);
    cv::cv2eigen(cv_T, eigen_T);

    Tbw = eigen_T;
    Rbw = eigen_R;

    cout << "the Tbw = " << endl << Tbw << endl;
    cout << "the Rbw = " << endl << Rbw << endl;

    width = static_cast<int>(fsSettings["width"]);
    height = static_cast<int>(fsSettings["height"]);

    undisKeyLine = fsSettings["undisKeyLine"];

    iterations = static_cast<int>(fsSettings["iterations"]);
    per_inliers = static_cast<int>(fsSettings["per_inliers"]);
    threshold = static_cast<double>(fsSettings["threshold"]);
    lambda = static_cast<double>(fsSettings["lambda"]);
    save = static_cast<int>(fsSettings["savefile"]);

    overlap_th = static_cast<double>(fsSettings["overlap_th"]);
    dist_th = static_cast<double>(fsSettings["dist_th"]);
    degree_th = static_cast<double>(fsSettings["degree_th"]);
    angle_th = static_cast<double>(fsSettings["angle_th"]);
    outlier_th = static_cast<double>(fsSettings["outlier_th"]);

    ROS_INFO("Finishing setting params for sliding window...");

}

void Estimator::clearState()
{
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        //line feature
        detectedLines[i].clear();

        if (pre_integrations[i] != nullptr)
            delete pre_integrations[i];
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    for (auto &it : all_image_frame)
    {
        if (it.second.pre_integration != nullptr)
        {
            delete it.second.pre_integration;
            it.second.pre_integration = nullptr;
        }
    }

    solver_flag = INITIAL;
    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();
    td = TD;


    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;
    relocalization_info = 0;

    drift_correct_r = Matrix3d::Identity();
    drift_correct_t = Vector3d::Zero();
}

void Estimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }


    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)
            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;

    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void Estimator::processImagewithLine(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const cv::Mat &image_show,
                                     const vector<pair<int, Eigen::Vector4d>> &lines2d, const std_msgs::Header &header)
{
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    if(f_manager.addFeaturesCheckParallax(frame_count, image, lines2d, td))  // add new point features
        marginalization_flag = MARGIN_OLD;
    else
        marginalization_flag = MARGIN_SECOND_NEW;

    ROS_DEBUG("this frame is--------------------%s", marginalization_flag ? "reject" : "accept");
    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    ROS_DEBUG("number of line feature: %d", f_manager.getLineFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header.stamp.toSec());
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(header.stamp.toSec(), imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if(frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }
    if (solver_flag == INITIAL)
    {
        if (frame_count == WINDOW_SIZE)
        {
            bool result = false;
            if (ESTIMATE_EXTRINSIC != 2 && (header.stamp.toSec() - initial_timestamp) > 0.1)
            {
                std::cout << "the initial Structure process once----------------" << std::endl;
                result = initialStructure();

                images_show[frame_count] = image_show;

                initial_timestamp = header.stamp.toSec();

            }
            if (result)
            {
                cout << "pass the initial phase------------------" << endl;
                solver_flag = NON_LINEAR;

                std_msgs::Header header_start = Headers[frame_count];

                for (unsigned int k = 0; k <= frame_count; ++k)
                {
                    cout << "the start optimization time stamp = ( " << k << " )" << Headers[k].stamp.toSec() <<  endl;
                }

                 initialLineFoVWindow();

                 updateLinePairInWindow();

                 f_manager.removeLineOutlier();
                 solveOdometry();

                 slideWindowWithLinesFoV();

                f_manager.removeFailures();
                ROS_INFO("Initialization finish!!!");
                ROS_INFO("Line Feature Initialization finish!!!");
                cout << "the solver flag = " << solver_flag << endl;

                last_R = Rs[WINDOW_SIZE];
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];
                last_P0 = Ps[0];

            }
            else
            {
                slideWindowWithLinesFoV();
            }

        }
        else
        {
            images_show[frame_count] = image_show;
            frame_count++;
        }

    }
    else
    {
        images_show[WINDOW_SIZE] = image_show;
        TicToc t_solve;

       vector2double();

        Eigen::Vector3d _Tic(para_Ex_Pose[0][0], para_Ex_Pose[0][1], para_Ex_Pose[0][2]);
        Eigen::Matrix3d _Ric = Quaterniond(para_Ex_Pose[0][6],
                                   para_Ex_Pose[0][3],
                                   para_Ex_Pose[0][4],
                                   para_Ex_Pose[0][5]).normalized().toRotationMatrix();

        UpdateLinesInFoV(frame_count, _Ric, _Tic);

        updateLinePairInWindow();

        f_manager.removeLineOutlier();

        solveOdometry();

        VisualizationWithTrackLines();

        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }
        TicToc t_margin;

        slideWindowWithLinesFoV();

        f_manager.removeFailures();
        ROS_DEBUG("marginalization costs: %fms", t_margin.toc());
      
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            key_poses.push_back(Ps[i]);
        }

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];

    }

}

void Estimator::UpdateLinesInFoV(int i, const Matrix3d &_Ric, const Vector3d &_Tic)
{
    line3dBody.clear();
    line3dCam.clear();
    projectedline.clear();

    Eigen::Matrix3d Ric = _Ric;
    Eigen::Vector3d Tic = _Tic;
    Eigen::Vector3d Tbi (para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
    Eigen::Quaterniond Qbi(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]);
    // transfer Q into R rotation matrix form
    Eigen::Matrix3d Rbi = Qbi.normalized().toRotationMatrix();

    float time_stamp = Headers[i].stamp.toSec();
    int GTIndex = 0;

    // get the whole extrinsic parameter
    Eigen::Matrix3d R = Ric.transpose() * Rbi.transpose() * Rbw;
    Eigen::Vector3d T = Ric.transpose() * (Rbi.transpose() * (Tbw - Tbi) - Tic);

    int height_up = -2 * WINDOW_SIZE;
    int height_down = 2 * WINDOW_SIZE + height;
    int width_left = -2 * WINDOW_SIZE;
    int width_right = 2 * WINDOW_SIZE + width;

    LinesInFoV.clear();

    for (size_t j = 0; j < lines3d_map.size(); ++j)
    {
        bool start_flag = false, end_flag = false;
        Eigen::Vector3d start_pt = Eigen::Vector3d(lines3d_map[j][0], lines3d_map[j][1], lines3d_map[j][2]);
        Eigen::Vector3d end_pt = Eigen::Vector3d(lines3d_map[j][3], lines3d_map[j][4], lines3d_map[j][5]);
        Line3D lineWorld(start_pt, end_pt);

        Eigen::Vector3d tf_start_pt = R * start_pt + T;    // projective from world frame into camera frame
        Eigen::Vector3d tf_end_pt = R * end_pt + T;

        double xx, yy, xx_, yy_;
        if((tf_start_pt[2] > 0) && (tf_end_pt[2] > 0))
        {
            xx = K(0, 0) * tf_start_pt[0] / tf_start_pt[2] + K(0, 2);    // image frame
            yy = K(1, 1) * tf_start_pt[1] / tf_start_pt[2] + K(1, 2);

            xx_ = K(0, 0) * tf_end_pt[0] / tf_end_pt[2] + K(0, 2);    // image frame
            yy_ = K(1, 1) * tf_end_pt[1] / tf_end_pt[2] + K(1, 2);

           if (xx > width_left && xx < (width_right - 1) && yy > height_up && yy < (height_down))
            {
                start_flag = true;
            }
            if (xx_ > width_left && xx_ < (width_right - 1) && yy_ > height_up && yy_ < (height_down))
            {
                end_flag = true;
            }
        }
        if (start_flag || end_flag)
        {
            LinesInFoV.push_back(lineWorld);
        }
    }

    WorldLinesInFOV[i] = LinesInFoV;
}

void Estimator::updateLinePairInWindow()
{

    vector2double();
    for ( auto &it_per_id : f_manager.linefeature)
    {
        int start_idx = it_per_id.start_frame;
        //if (!(it_per_id.credible_matching))
        {
            // rematching this line_id in every frame
            for (auto &it_per_frame :  it_per_id.linefeature_per_frame)
            {
                Line2D detectLine = it_per_frame.line;
                pair<Eigen::Vector3f, pair<Line2D, Line3D>> linePair = LineCorrespondenceInFrame(start_idx, detectLine);
                it_per_frame.errA = linePair.first[0];
                it_per_frame.errD = linePair.first[1];
                it_per_frame.overlap = linePair.first[2];
                it_per_frame.projectedLine = linePair.second.first;
                it_per_frame.lineWorld = linePair.second.second;
                it_per_frame.use_flag = true;
                start_idx++;
                if (linePair.first[0] == -1)
                {
                    it_per_frame.credible_line = false;
                }
                else
                {
                    it_per_frame.credible_line = true;
                }
            }
        }
    }
}

void Estimator::initialLineFoVWindow()
{
    TicToc t_iniL;
    vector2double();
    Eigen::Vector3d _Tic(para_Ex_Pose[0][0], para_Ex_Pose[0][1], para_Ex_Pose[0][2]);
    Eigen::Matrix3d _Ric = Quaterniond(para_Ex_Pose[0][6],
                                       para_Ex_Pose[0][3],
                                       para_Ex_Pose[0][4],
                                       para_Ex_Pose[0][5]).normalized().toRotationMatrix();
    LinesInFoV.clear();
    for (int i = 0; i <= frame_count; ++i)
    {
        UpdateLinesInFoV(i, _Ric, _Tic);
    }
}

void Estimator::updateLineConstraint()
{

    TicToc t_iniL;
    vector2double();  // get the initial state
    // get the extrinsic para between camera and IMU
    Eigen::Vector3d _Tic(para_Ex_Pose[0][0], para_Ex_Pose[0][1], para_Ex_Pose[0][2]);
    Eigen::Matrix3d _Ric = Quaterniond(para_Ex_Pose[0][6],
                                       para_Ex_Pose[0][3],
                                       para_Ex_Pose[0][4],
                                       para_Ex_Pose[0][5]).normalized().toRotationMatrix();

    // get the initial 2D-3D line correspondences for the line optimization
    for (int i = 0; i <= frame_count; ++i)
    {
        lines3dBody[i].clear();
        lines3dCam[i].clear();
        projectedLines[i].clear();
        // get the potential 3d lines

        UpdateLinesInFoV(i, _Ric, _Tic);

        lines3dBody[i] = line3dBody;
        lines3dCam[i] = line3dCam;
        projectedLines[i] = projectedline;

        detectedline.clear();
        detectedline = detectedLines[i];

        LineCorrespondence();

        lineMatches[i] = lineMatch;
        lineMatchesFilter[i] = lineMatchFilter;

        //
        line3dBody.clear();
        line3dCam.clear();
        projectedline.clear();
        detectedline.clear();
        lineMatch.clear();
        lineMatchFilter.clear();
    }
}


Eigen::Vector4d Estimator::CalAngleDistInCamera(const Line3D &_lineC, const Line2D &_projectedL,
                                     const Line2D &_detectedL)
{
    Line3D lineCam = _lineC;
    Line2D projectedL = _projectedL;
    Line2D detectedL = _detectedL;
    double alpha, theta, phi, beta;
    Eigen::Vector4d MatchFactorA = Vector4d{PI, PI, PI, PI};  // lambda * theta + (1-lambda) * phi , theta, phi
    Eigen::Vector3d circleNormal = detectedL.ComputeCircleNormal(K);

    theta = abs(PI / 2 - acos(abs(lineCam.Direction.transpose() * circleNormal)));

    beta = acos(abs(detectedL.Direction.transpose() * projectedL.Direction));

    Vector2d nearestP = projectedL.Point2Flined(detectedL.PtrMid);
    Vector3d nearestPN = Vector3d(nearestP[0], nearestP[1], 1);
    Vector3d NearP = (K.inverse() * nearestPN).normalized();
    Vector3d midP = Vector3d(detectedL.PtrMid[0], detectedL.PtrMid[1], 1);
    Vector3d MidP = (K.inverse() * midP).normalized();
    phi = acos(abs(NearP.transpose() * MidP));

    double lambda2 = lambda;
    alpha = (lambda / 2) * theta + (1 - lambda) * phi + (lambda2 / 2) * beta;

    // the angle on the normalized camera plane
    Vector2d directionD, directionP;
    double lengthD, lengthP;
    Vector2d sD, eD, sP, eP;
    sD = detectedL.LineObsN.head(2);
    eD = detectedL.LineObsN.tail(2);
    directionD = (sD - eD);
    lengthD = directionD.norm();
    directionD = directionD / lengthD;
    sP(0) = lineCam.PtrStart(0) / lineCam.PtrStart(2);
    sP(1) = lineCam.PtrStart(1) / lineCam.PtrStart(2);
    eP(0) = lineCam.PtrEnd(0) / lineCam.PtrEnd(2);
    eP(1) = lineCam.PtrEnd(1) / lineCam.PtrEnd(2);
    directionP = sP - eP;
    lengthP = directionP.norm();
    directionP = directionP / lengthP;
    alpha = acos(abs(directionP.transpose() * directionD));

    sP = projectedL.LineObsN.head(2);
    eP = projectedL.LineObsN.tail(2);
    directionP = sP - eP;
    lengthP = directionP.norm();
    directionP = directionP / lengthP;
    alpha = acos(abs(directionP.transpose() * directionD));

    MatchFactorA = Vector4d{alpha, theta, phi, beta};

    if (isnan(alpha) || isnan(theta) || isnan(phi) || isnan(beta))
        MatchFactorA = Vector4d{PI, PI, PI, PI};

    return MatchFactorA;
}

double Estimator::CalAngleDist(const Line2D &_projectedL,
                               const Line2D &_detectedL)
{
    Line2D projectedL = _projectedL;
    Line2D detectedL = _detectedL;
    double beta = PI;

    beta = acos(abs(detectedL.Direction.transpose() * projectedL.Direction));

    if (isnan(beta))
        beta = PI;
    return beta;
}

Eigen::Vector2d Estimator::CalEulerDist(const Line2D &_projectedL, const Line2D &_detectedL)
{
    Line2D projectedL = _projectedL;
    Line2D detectedL = _detectedL;
    Vector2d MatchFactorD;
    int sampleLen = 20;
    int sampleNum = 10;
    double lengthM = detectedL.Length;
    double lengthP = projectedL.Length;
    double lengthMin;
    double distance = 10000.0;
    double overlap_ratio1 = 0.0, overlap_ratio2 = 0.0, overlap_ratio = 0.0;
    double angle = PI;

    MatchFactorD = Vector2d(distance, overlap_ratio);

    Line2D line1, line2;
    if (lengthM <= lengthP)
    {
        lengthMin = lengthM;
        line1 = detectedL;
        line2 = projectedL;
    }
    else
    {
        lengthMin = lengthP;
        line1 = projectedL;
        line2 = detectedL;
    }
    overlap_ratio1 = ((line2.Point2Flined(line1.PtrStart)
                      -line2.Point2Flined(line1.PtrEnd)).norm()) / line2.Length;
    overlap_ratio = overlap_ratio1;

    double point_x = line1.PtrStart[0], point_y = line1.PtrStart[1];
    double len_x = line1.PtrStart[0] - line1.PtrEnd[0];
    double len_y = line1.PtrStart[1] - line1.PtrEnd[1];
    double step_x = len_x / sampleNum;
    double step_y = len_y / sampleNum;
    distance = 0.0;

    for (int i = 0; i < sampleNum; ++i)
    {
        double x = point_x + i * step_x;
        double y = point_y + i * step_y;
        distance = distance +  abs(line2.A * x + line2.B * y + line2.C) / line2.A2B2;
    }
    distance = distance + 1 * abs(line2.A * line1.PtrStart[0] + line2.B * line1.PtrStart[1] + line2.C) / line2.A2B2;
    distance = distance + 1 * abs(line2.A * line1.PtrEnd[0] + line2.B * line1.PtrEnd[1] + line2.C) / line2.A2B2;
    distance = distance / (sampleNum + 2);
    MatchFactorD = Vector2d{distance, overlap_ratio};
    if (isnan(distance) || isnan(overlap_ratio))
        MatchFactorD = Vector2d{10000.0, 0.0};

    return MatchFactorD;
}

pair<Eigen::Vector3f ,pair<Line2D, Line3D>> Estimator::LineCorrespondenceInFrame(const int &_frame_index, const Line2D &_detectLine)
{
    int frame_index = _frame_index;
    Line2D detectLine = _detectLine;
    vector<Line3D> linesInThisFov = WorldLinesInFOV[frame_index];

    float time_stamp = Headers[frame_index].stamp.toSec();

    Eigen::Vector3d Tic(para_Ex_Pose[0][0], para_Ex_Pose[0][1], para_Ex_Pose[0][2]);
    Eigen::Matrix3d Ric = Quaterniond(para_Ex_Pose[0][6],
                                       para_Ex_Pose[0][3],
                                       para_Ex_Pose[0][4],
                                       para_Ex_Pose[0][5]).normalized().toRotationMatrix();

    Eigen::Vector3d Tbi (para_Pose[frame_index][0], para_Pose[frame_index][1], para_Pose[frame_index][2]);
    Eigen::Quaterniond Qbi(para_Pose[frame_index][6], para_Pose[frame_index][3], para_Pose[frame_index][4], para_Pose[frame_index][5]);
    // transfer Q into R rotation matrix form
    Eigen::Matrix3d Rbi = Qbi.normalized().toRotationMatrix();

    // get the whole extrinsic parameter
    Eigen::Matrix3d R = Ric.transpose() * Rbi.transpose() * Rbw;
    Eigen::Vector3d T = Ric.transpose() * (Rbi.transpose() * (Tbw - Tbi) - Tic);

    Line2D projectedLine;
    Line3D choose_line;
    int choose_index = -1;
    Line2D temp_line;

    Eigen::Vector3f error;
    error.setZero();
    float min_dist = 10000.0;

    if (linesInThisFov.size() == 0)
    {
        error[0] = -1;
        error[1] = -1;
        error[2] = -1;
        Vector3d fake_point = detectLine.hPtrStart;
        Line3D fake_line(fake_point, fake_point);
        pair<Line2D, Line3D> linePair = make_pair(detectLine, fake_line);
      //  ROS_INFO("after make pair between two lines");
        return make_pair(error, linePair);
    }

    for (unsigned int i = 0; i < linesInThisFov.size(); ++i)
    {
        Eigen::Vector2d MatchFactorD;
        float overlap = 0.0;
        float distance = 10000.0;

        Line3D lineW = linesInThisFov[i];
        Eigen::Vector3d start_pt = lineW.PtrStart;
        Eigen::Vector3d end_pt = lineW.PtrEnd;

        bool start_flag = false, end_flag = false;

        Eigen::Vector3d tf_start_pt = R * start_pt + T;    // projective from world frame into camera frame
        Eigen::Vector3d tf_end_pt = R * end_pt + T;

        float xx, yy, xx_, yy_;

        if (tf_start_pt[2] > 0 && tf_end_pt[2] > 0)
        {
            xx = K(0,0) * tf_start_pt[0] / tf_start_pt[2] + K(0,2);    // image frame
            yy = K(1,1) * tf_start_pt[1] / tf_start_pt[2] + K(1,2);
            xx_ = K(0,0) * tf_end_pt[0] / tf_end_pt[2] + K(0,2);    // image frame
            yy_ = K(1,1) * tf_end_pt[1] / tf_end_pt[2] + K(1,2);

            if (xx > 0 && xx < width - 1 && yy > 0 && yy < height - 1)
                start_flag = true;
            if (xx_ > 0 && xx_ < width - 1 && yy_ > 0 && yy_ < height - 1)
                end_flag = true;
        }

        if (start_flag && end_flag) // both end points are in FOV
        {

            temp_line = Line2D(Eigen::Vector4d(xx, yy, xx_, yy_));
            double angle = CalAngleDist(temp_line, detectLine);
            if (angle > angle_th)
                continue;
            MatchFactorD = CalEulerDist(temp_line, detectLine);
            distance = MatchFactorD[0];
            overlap = MatchFactorD[1];
            // compare the distance matching factor
            if (overlap < overlap_th)
                continue;
            if (distance < min_dist)
            {
                min_dist = distance;
                choose_index = i;
                projectedLine = temp_line;
                error[0] = angle;
                error[1] = min_dist;
                error[2] = overlap;
            }
        }
        else if (start_flag && (!end_flag))
        {
            Eigen::Vector3d dirvec = tf_end_pt - tf_start_pt;
            double t = 0.9;
            bool inFOV = true;
            bool findPointE = false;
            double x = 0.0, y = 0.0;
            while (t > 0)
            {
                Eigen::Vector3d temp_tf_end_pt = tf_start_pt + t * dirvec;
                if (temp_tf_end_pt[2] > 0)
                {
                    x = K(0, 0) * temp_tf_end_pt[0] / temp_tf_end_pt[2] + K(0, 2);
                    y = K(1, 1) * temp_tf_end_pt[1] / temp_tf_end_pt[2] + K(1, 2);
                    if (x > 0 && x < (width - 1) && y > 0 && y < (height - 1))
                    {
                        findPointE = true;
                        break;
                    }
                    else
                        t = t - 0.1;
                }
                else
                    t = t - 0.1;
            }
            if(findPointE)
            {

                temp_line = Line2D(Eigen::Vector4d(xx, yy, x, y));
                double angle = CalAngleDist(temp_line, detectLine);
                if (angle > angle_th)
                    continue;
                MatchFactorD = CalEulerDist(temp_line, detectLine);
                distance = MatchFactorD[0];
                overlap = MatchFactorD[1];
                // compare the distance matching factor
                if (overlap < overlap_th)
                    continue;
                if (distance < min_dist)
                {
                    min_dist = distance;
                    choose_index = i;
                    projectedLine = temp_line;
                    error[0] = angle;
                    error[1] = min_dist;
                    error[2] = overlap;
                }
            }
        }
        else if (end_flag && (!start_flag))
        {
            Eigen::Vector3d dirvec = tf_start_pt- tf_end_pt;
            double t = 0.9;
            bool inFOV = true;
            bool findPointS = false;
            double x = 0.0, y = 0.0;
            while (t > 0)
            {
                Eigen::Vector3d temp_tf_start_pt = tf_end_pt + t * dirvec;
                if (temp_tf_start_pt[2] > 0)
                {
                    x = K(0, 0) * temp_tf_start_pt[0] / temp_tf_start_pt[2] + K(0, 2);
                    y = K(1, 1) * temp_tf_start_pt[1] / temp_tf_start_pt[2] + K(1, 2);
                    if (x > 0 && x < (width - 1) && y > 0 && y < (height - 1))
                    {
                        findPointS = true;
                        break;
                    }
                    else
                        t = t - 0.1;
                }
                else
                    t = t - 0.1;
            }
            if(findPointS)
            {

                temp_line = Line2D(Eigen::Vector4d(x, y, xx_, yy_));
                double angle = CalAngleDist(temp_line, detectLine);
                if (angle > angle_th)
                    continue;
                MatchFactorD = CalEulerDist(temp_line, detectLine);
                distance = MatchFactorD[0];
                overlap = MatchFactorD[1];
                // compare the distance matching factor
                if (overlap < overlap_th)
                    continue;
                if (distance < min_dist)
                {
                    min_dist = distance;
                    choose_index = i;
                    projectedLine = temp_line;
                    error[0] = angle;
                    error[1] = min_dist;
                    error[2] = overlap;
                }
            }
        }

    }

    if (choose_index == -1)
    {
        error[0] = -1;
        error[1] = -1;
        error[2] = -1;

        pair<Line2D, Line3D> linePair = make_pair(detectLine, linesInThisFov[0]);

        return make_pair(error, linePair);
    }
    // choose the best matching line
    choose_line = linesInThisFov[choose_index];
    pair<Line2D, Line3D> linePair = make_pair(projectedLine, choose_line);
    return make_pair(error, linePair);
    // maybe return a pair not a vector<pair<>> ??!  check it!!!

}

void Estimator::LineCorrespondence()
{
    lineMatch.clear();
    vector<Line3D> _lineBody = line3dBody;
    vector<Line3D> _lineCam = line3dCam;
    vector<Line2D> _projectedL = projectedline;
    vector<Line2D> _detectedL = detectedline;
    Eigen::Vector2d MatchFactorD;
    Eigen::Vector4d MatchFactorA;

    for (int i = 0; i < _detectedL.size(); ++i)
    {
        int idx = 0;
        double min_dist = 10000.0;
        double min_angle = 10.0;
        Vector6d matchdata;
        matchdata[0] = min_dist;
        matchdata[1] = 0;

        //cout << "the projected line number = " << _projectedL.size();
        for (int j = 0; j < _projectedL.size(); ++j)
        {
            MatchFactorA = CalAngleDistInCamera(_lineCam[j], _projectedL[j], _detectedL[i]);

            if (MatchFactorA[3] >= angle_th)
            {
                continue;
            }
            MatchFactorD = CalEulerDist(_projectedL[j], _detectedL[i]);

            if(MatchFactorD[1] < overlap_th)
            {
                continue;
            }
            if (MatchFactorD[0] < min_dist)    // consider the euler dist and angle
            {
                //   min_angle = MatchFactorA[0];
                // 0 dist,
                // 1 overlap,
                // 2 alpha: the weighted angle
                // 3 theta: the angle on camera frame
                // 4 phi: the midpoint and nearest angle
                // 5 beta: the angle on image frame
                min_dist = MatchFactorD[0];
                matchdata << MatchFactorD[0], MatchFactorD[1], MatchFactorA[0],
                        MatchFactorA[1], MatchFactorA[2], MatchFactorA[3];
                idx = j;
            }
        }

        if (min_dist < dist_th)
        {
            PairsMatch match(i, _lineBody[idx], _lineCam[idx], _projectedL[idx], _detectedL[i]);
            match.criteria = matchdata;
            double Weight, w_1, w_2, w_3;
            w_1 = 1 / sqrt(matchdata[0] / 10) * 10;
            w_2 = matchdata[1];
            w_3 = 1 - (matchdata[5] / PI * 180) / 5;
            Weight = w_1 + w_2 + w_3;
            match.weight = Weight;
            match.weight = 0.02;
            lineMatch.push_back(match);
        }
    }

    CorrespondenceFilter();

}

void Estimator::CorrespondenceFilter()
{
    lineMatchFilter.clear();
    double AveAngle = 0.0;
    double AveDist = 0.0;
    double AveOverlap = 0.0;

    vector<PairsMatch> InputMatches_ = lineMatch;

    int RepetitiveCount = 0;

    for (size_t i = 0; i < InputMatches_.size(); ++i)
    {
        Line2D projectedL = InputMatches_[i].projectedlineP;
        Eigen::Vector2d startP = projectedL.PtrStart;
        Eigen::Vector2d endP = projectedL.PtrEnd;
        if (!InputMatches_[i].repetitive)
        {
            for (size_t j = i + 1; j < InputMatches_.size(); ++j)
            {
                Line2D projectedL_ = InputMatches_[j].projectedlineP;
                Eigen::Vector2d startP_ = projectedL_.PtrStart;
                Eigen::Vector2d endP_ = projectedL_.PtrEnd;
                if (startP(0) == startP_(0) && startP(1) == startP_(1) && endP(0) == endP_(0) && endP(1) == endP_(1))
                {
                    
                    double dist = InputMatches_[i].criteria[0];
                    double dist_ = InputMatches_[j].criteria[0];
                    if (dist > dist_)
                    {
                        
                        InputMatches_[i].repetitive = true;
                    }
                    else
                    {
                        
                        InputMatches_[j].repetitive = true;
                    }
                    RepetitiveCount += 1;
                }
            }
        }
    }

    for (int i = 0; i < RepetitiveCount; ++i)
    {
        for (int j = 0; j < InputMatches_.size(); ++j)
        {
            if (InputMatches_[j].repetitive)
            {
                InputMatches_.erase(InputMatches_.begin() + j);
                break;
            }
        }
    }
    sort(InputMatches_.begin(), InputMatches_.end(), Compare);

    for (size_t i = 0; i < InputMatches_.size(); ++i)
    {
        AveAngle += InputMatches_[i].criteria[5];
        AveDist += InputMatches_[i].criteria[0];
        AveOverlap += InputMatches_[i].criteria[1];
    }
    AveAngle = AveAngle / InputMatches_.size();
    AveDist = AveDist / InputMatches_.size();
    AveOverlap = AveOverlap / InputMatches_.size();

    double Dist_in = AveDist * outlier_th;
    double Angele_in = AveAngle * outlier_th;
    double Overlap_in = AveOverlap * outlier_th;

    for (size_t i = 0; i < InputMatches_.size(); ++i)
    {
        if(InputMatches_[i].criteria[0] <= Dist_in * 1.6  && InputMatches_[i].criteria[5] <= Angele_in * 1.6)
        {

            Vector3d start = InputMatches_[i].line3dBodyP.PtrStart;
            Vector3d end = InputMatches_[i].line3dBodyP.PtrEnd;

            lineMatchFilter.push_back(InputMatches_[i]);

        }
    }

    int MaxMatch = 60;
    if (lineMatchFilter.size() > MaxMatch)
    {
        lineMatchFilter.erase(lineMatchFilter.begin() + MaxMatch, lineMatchFilter.end());
    }

}


void Estimator::VisualizationProLinesInFoV()
{
    cv::Mat img, image;
    image = images_show[frame_count];
    if (image.channels() != 3)
    {
        cv::cvtColor(image, img, cv::COLOR_GRAY2BGR);
    }
    else
    {
        img = image;
    }
    cv::Point startPointE;
    cv::Point endPointE;
    Line2D linePro;

    Eigen::Vector3d Tic(para_Ex_Pose[0][0], para_Ex_Pose[0][1], para_Ex_Pose[0][2]);
    Eigen::Matrix3d Ric = Quaterniond(para_Ex_Pose[0][6],
                                      para_Ex_Pose[0][3],
                                      para_Ex_Pose[0][4],
                                      para_Ex_Pose[0][5]).normalized().toRotationMatrix();

    float time_stamp = Headers[frame_count].stamp.toSec();
    int GTIndex = 0;
    GTIndex = getGTIndex(time_stamp, BeginIndexSW);

    Eigen::Vector3d Tbi(benchmark[GTIndex].px, benchmark[GTIndex].py, benchmark[GTIndex].pz);
    Eigen::Vector3d rpy(benchmark[GTIndex].roll, benchmark[GTIndex].pitch, benchmark[GTIndex].yaw);
    Eigen::Matrix3d Rbi_ = rpy2R(rpy);
    Eigen::Matrix3d Rbi = Rbi_;

    Eigen::Matrix3d Rbw_;
    Rbw_ << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    Eigen::Vector3d Tbw_;
    Tbw_ << 0.0, 0.0, -2.0;


    // get the whole extrinsic parameter
    Eigen::Matrix3d R = Ric.transpose() * Rbi.transpose() * Rbw_;
    Eigen::Vector3d T = Ric.transpose() * (Rbi.transpose() * (Tbw_ - Tbi) - Tic);

    vector<Line3D> linesInFov = WorldLinesInFOV[frame_count];
    for (unsigned int i = 0; i < linesInFov.size(); ++i)
    {
        float xx, yy, xx_, yy_;
        Line3D lineW = linesInFov[i];
        Eigen::Vector3d start_pt = lineW.PtrStart;
        Eigen::Vector3d end_pt = lineW.PtrEnd;

        bool start_flag = false, end_flag = false;

        Eigen::Vector3d tf_start_pt = R * start_pt + T;    // projective from world frame into camera frame
        Eigen::Vector3d tf_end_pt = R * end_pt + T;

        if (tf_start_pt[2] > 0 && tf_end_pt[2] > 0)
        {
            xx = K(0, 0) * tf_start_pt[0] / tf_start_pt[2] + K(0, 2);    // image frame
            yy = K(1, 1) * tf_start_pt[1] / tf_start_pt[2] + K(1, 2);
            xx_ = K(0, 0) * tf_end_pt[0] / tf_end_pt[2] + K(0, 2);    // image frame
            yy_ = K(1, 1) * tf_end_pt[1] / tf_end_pt[2] + K(1, 2);

            if (xx > 0 && xx < width - 1 && yy > 0 && yy < height - 1)
                start_flag = true;
            if (xx_ > 0 && xx_ < width - 1 && yy_ > 0 && yy_ < height - 1)
                end_flag = true;
        }

        if (start_flag && end_flag)
        {
            startPointE = cv::Point(xx, yy);
            endPointE = cv::Point(xx_, yy_);
            cv::line(img, startPointE, endPointE, cv::Scalar(0, 255, 0), 2, 8);  // green
        }
    }

    cv::imshow("the line in FoV", img);
    cv::waitKey(1);

}

void Estimator::VisualizationWithTrackLines()
{
    cv::Mat img1, image;
    image = images_show[frame_count];
    if (image.channels() != 3)
    {
        cv::cvtColor(image, img1, cv::COLOR_GRAY2BGR);
    }
    else
    {
        img1 = image;
        //    ROS_INFO("image channels is 3");
    }

    cv::Point startPointD;
    cv::Point endPointD;
    cv::Point startPointP;
    cv::Point endPointP;

    Line2D lineDet;
    Line2D linePro;

    for (auto &it_per_id : f_manager.linefeature)
    {
        int end_frame = it_per_id.endFrame();
        if (end_frame == frame_count)
        {
            lineFeaturePerFrame show_line = it_per_id.linefeature_per_frame.back();
            lineDet = show_line.line;
            linePro = show_line.projectedLine;

            startPointD = cv::Point(lineDet.PtrStart[0], lineDet.PtrStart[1]);
            endPointD = cv::Point(lineDet.PtrEnd[0], lineDet.PtrEnd[1]);
            startPointP = cv::Point(linePro.PtrStart[0], linePro.PtrStart[1]);
            endPointP = cv::Point(linePro.PtrEnd[0], linePro.PtrEnd[1]);
            if (it_per_id.credible_matching && show_line.credible_line)
            {
                cv::line(img1, startPointD, endPointD, cv::Scalar(0, 0, 255), 2, 8);  // red detect
                cv::line(img1, startPointP, endPointP, cv::Scalar(0, 255, 0), 2, 8);  // green
            }
        }

    }

    cv::imshow("the matching line result", img1);
    cv::waitKey(1);

}

void Estimator::VisualizationMatchedLines()
{
    cv::Mat img1, img2, image;
    image = images_show[frame_count];
    if (image.channels() != 3)
    {
        cv::cvtColor(image, img1, cv::COLOR_GRAY2BGR);
        cv::cvtColor(image, img2, cv::COLOR_GRAY2BGR);
    }
    else
    {
        img1 = image;
        img2 = image;
        //    ROS_INFO("image channels is 3");
    }

    cv::Point startPointM;
    cv::Point endPointM;
    cv::Point startPointE;
    cv::Point endPointE;

    Line2D lineDet;
    Line2D linePro;

    vector<PairsMatch> linePairShow = lineMatchesFilter[frame_count];
    for (int i = 0; i < linePairShow.size(); ++i)
    {
        lineDet = linePairShow[i].detectedlineP;
        linePro = linePairShow[i].projectedlineP;
        startPointM = cv::Point(lineDet.PtrStart[0], lineDet.PtrStart[1]);
        endPointM = cv::Point(lineDet.PtrEnd[0], lineDet.PtrEnd[1]);
        startPointE = cv::Point(linePro.PtrStart[0], linePro.PtrStart[1]);
        endPointE = cv::Point(linePro.PtrEnd[0], linePro.PtrEnd[1]);
        cv::line(img1, startPointM, endPointM, cv::Scalar(255, 0, 0), 2, 8);  // blue 3d
        cv::line(img1, startPointE, endPointE, cv::Scalar(0, 0, 255), 2, 8);  // red 2d
    }

    cv::imshow("show matched line filter", img1);
    waitKey(1);

}


bool Estimator::initialStructure()
{
    TicToc t_sfm;
    //check imu observibility
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        //ROS_WARN("IMU variation %f!", var);
        if(var < 0.25)
        {
            ROS_INFO("IMU excitation not enouth!");
            //return false;
        }
    }
    // global sfm
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    }
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    if (!relativePose(relative_R, relative_T, l))
    {
        ROS_INFO("Not enough features or parallax; Move device around");
        return false;
    }
    GlobalSFM sfm;
    if(!sfm.construct(frame_count + 1, Q, T, l,
              relative_R, relative_T,
              sfm_f, sfm_tracked_points))
    {
        ROS_DEBUG("global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if((frame_it->first) == Headers[i].stamp.toSec())
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if((frame_it->first) > Headers[i].stamp.toSec())
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_points.find(feature_id);
                if(it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        if(pts_3_vector.size() < 6)
        {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }
        if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            ROS_DEBUG("solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }
    if (visualInitialAlign())
        return true;
    else
    {
        ROS_INFO("misalign visual structure with IMU");
        return false;
    }

}

bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;
    //solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if(!result)
    {
        ROS_DEBUG("solve g failed!");
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i].stamp.toSec()].R;
        Vector3d Pi = all_image_frame[Headers[i].stamp.toSec()].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i].stamp.toSec()].is_key_frame = true;
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < dep.size(); i++)
        dep[i] = -1;
    f_manager.clearDepth(dep);

    //triangulat on cam pose , no tic
    Vector3d TIC_TMP[NUM_OF_CAM];
    for(int i = 0; i < NUM_OF_CAM; i++)
        TIC_TMP[i].setZero();
    ric[0] = RIC[0];
    f_manager.setRic(ric);
    f_manager.triangulate(Ps, &(TIC_TMP[0]), &(RIC[0]));

    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if(frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth *= s;
    }

    Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    ROS_DEBUG_STREAM("g0     " << g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());

    return true;
}

bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;

            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            if(average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}

void Estimator::solveOdometry()
{
    if (frame_count < WINDOW_SIZE)
        return;
    if (solver_flag == NON_LINEAR)
    {
        TicToc t_tri;
        f_manager.triangulate(Ps, tic, ric);
        ROS_DEBUG("triangulation costs %f", t_tri.toc());

        OptimizationWithLine();
      //  ROS_INFO("OptimizationWithLine finished!!!!!");

    }
}

void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();

        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();

        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();

        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);
    if (ESTIMATE_TD)
        para_Td[0][0] = td;
}

void Estimator::double2vector()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }
    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                      para_Pose[0][3],
                                                      para_Pose[0][4],
                                                      para_Pose[0][5]).toRotationMatrix());
    double y_diff = origin_R0.x() - origin_R00.x();
    //TODO
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
    {
        ROS_DEBUG("euler singular point!");
        rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                       para_Pose[0][3],
                                       para_Pose[0][4],
                                       para_Pose[0][5]).toRotationMatrix().transpose();
    }

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {

        Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();

        Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                para_Pose[i][1] - para_Pose[0][1],
                                para_Pose[i][2] - para_Pose[0][2]) + origin_P0;

        Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                    para_SpeedBias[i][1],
                                    para_SpeedBias[i][2]);

        Bas[i] = Vector3d(para_SpeedBias[i][3],
                          para_SpeedBias[i][4],
                          para_SpeedBias[i][5]);

        Bgs[i] = Vector3d(para_SpeedBias[i][6],
                          para_SpeedBias[i][7],
                          para_SpeedBias[i][8]);
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d(para_Ex_Pose[i][0],
                          para_Ex_Pose[i][1],
                          para_Ex_Pose[i][2]);
        ric[i] = Quaterniond(para_Ex_Pose[i][6],
                             para_Ex_Pose[i][3],
                             para_Ex_Pose[i][4],
                             para_Ex_Pose[i][5]).toRotationMatrix();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);
    if (ESTIMATE_TD)
        td = para_Td[0][0];


    // relative info between two loop frame
    if(relocalization_info)
    {
        Matrix3d relo_r;
        Vector3d relo_t;
        relo_r = rot_diff * Quaterniond(relo_Pose[6], relo_Pose[3], relo_Pose[4], relo_Pose[5]).normalized().toRotationMatrix();
        relo_t = rot_diff * Vector3d(relo_Pose[0] - para_Pose[0][0],
                                     relo_Pose[1] - para_Pose[0][1],
                                     relo_Pose[2] - para_Pose[0][2]) + origin_P0;
        double drift_correct_yaw;
        drift_correct_yaw = Utility::R2ypr(prev_relo_r).x() - Utility::R2ypr(relo_r).x();
        drift_correct_r = Utility::ypr2R(Vector3d(drift_correct_yaw, 0, 0));
        drift_correct_t = prev_relo_t - drift_correct_r * relo_t;
        relo_relative_t = relo_r.transpose() * (Ps[relo_frame_local_index] - relo_t);
        relo_relative_q = relo_r.transpose() * Rs[relo_frame_local_index];
        relo_relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs[relo_frame_local_index]).x() - Utility::R2ypr(relo_r).x());
        //cout << "vins relo " << endl;
        //cout << "vins relative_t " << relo_relative_t.transpose() << endl;
        //cout << "vins relative_yaw " <<relo_relative_yaw << endl;
        relocalization_info = 0;

    }
}

bool Estimator::failureDetection()
{
    if (f_manager.last_track_num < 2)
    {
        ROS_INFO(" little feature %d", f_manager.last_track_num);
        //return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        ROS_INFO(" big translation");
        return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        ROS_INFO(" big z translation");
        return true;
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        ROS_INFO(" big delta_angle ");
        //return true;
    }
    return false;
}

void Estimator::OptimizationWithLine()
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if (!ESTIMATE_EXTRINSIC)
        {
            ROS_DEBUG("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
        else
            ROS_DEBUG("estimate extinsic param");
    }

    if (ESTIMATE_TD)
    {
        problem.AddParameterBlock(para_Td[0], 1);
        //problem.SetParameterBlockConstant(para_Td[0]);
    }

    TicToc t_whole, t_prepare;
    vector2double();

    // prior factor

        if (last_marginalization_info)
        {
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            problem.AddResidualBlock(marginalization_factor, NULL,
                                     last_marginalization_parameter_blocks);
        }

    // IMU factor
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL,
                                 para_Pose[i], para_SpeedBias[i],
                                 para_Pose[j], para_SpeedBias[j]);
    }

    // point feature factor
    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i == imu_j)
            {
                continue;
            }
            Vector3d pts_j = it_per_frame.point;
            if (ESTIMATE_TD)
            {
                ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                  it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                                                                  it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
            }
            else
            {
                ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);
            }
            f_m_cnt++;
        }
    }

    // line feature optimization based on tracking strategy
    int effective_number = 0;
    if (1)
    {
        Eigen::Vector3d _Tic(para_Ex_Pose[0][0], para_Ex_Pose[0][1], para_Ex_Pose[0][2]);
        Eigen::Matrix3d _Ric = Quaterniond(para_Ex_Pose[0][6],
                                           para_Ex_Pose[0][3],
                                           para_Ex_Pose[0][4],
                                           para_Ex_Pose[0][5]).normalized().toRotationMatrix();
        int line_m_cnt = 0;

        bool flag_use_line = false;

        for (auto &it_per_id : f_manager.linefeature)
        {

            it_per_id.used_num = it_per_id.linefeature_per_frame.size();
            if (!( it_per_id.used_num >=2 && it_per_id.start_frame < WINDOW_SIZE - 2)) //
            {
                continue;
            }

            if (!(it_per_id.credible_matching))
            {
                for (unsigned int k = 0; k < it_per_id.statistics_error_per_id.size(); k++)
                {
                    cout << it_per_id.statistics_error_per_id[k] << endl;
                }
                continue;
            }

            int line_feature_frame = it_per_id.start_frame - 1;

            int line_feature_index = -1;
            for (auto &it_per_frame : it_per_id.linefeature_per_frame)
            {
                line_feature_index++;

                line_feature_frame++;

                if (!(it_per_frame.credible_line))
                {
                    effective_number++;
                    continue;

                }
                if (!(it_per_frame.use_flag))
                {
                    effective_number++;
                    continue;
                }

                if (it_per_frame.errD > dist_th)
                {
                    effective_number++;
                  continue;
                }

                Vector3d line_param(it_per_frame.line.A, it_per_frame.line.B, it_per_frame.line.C);
                const Eigen::Vector3d ptr_start = Rbw * it_per_frame.lineWorld.PtrStart + Tbw;
                const Eigen::Vector3d ptr_end = Rbw * it_per_frame.lineWorld.PtrEnd + Tbw;
                LineProjectionFactor *line_factor = new LineProjectionFactor(ptr_start, ptr_end, line_param,
                                                                             K, _Ric, _Tic);

                ceres::LocalParameterization *line_local_parameterization = new PoseLocalParameterization();
                problem.AddParameterBlock(para_Pose[line_feature_frame], SIZE_POSE, line_local_parameterization);

                problem.AddResidualBlock(line_factor, loss_function, para_Pose[line_feature_frame]);

                flag_use_line = true;

            }

        }
        if (flag_use_line == true)
            useful_frame++;

    }

    ROS_DEBUG("prepare for ceres: %f", t_prepare.toc());

    if(relocalization_info)
    {
        //printf("set relocalization factor! \n");
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(relo_Pose, SIZE_POSE, local_parameterization);
        int retrive_feature_index = 0;
        int feature_index = -1;
        for (auto &it_per_id : f_manager.feature)
        {
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;
            ++feature_index;
            int start = it_per_id.start_frame;
            if(start <= relo_frame_local_index)
            {
                while((int)match_points[retrive_feature_index].z() < it_per_id.feature_id)
                {
                    retrive_feature_index++;
                }
                if((int)match_points[retrive_feature_index].z() == it_per_id.feature_id)
                {
                    Vector3d pts_j = Vector3d(match_points[retrive_feature_index].x(), match_points[retrive_feature_index].y(), 1.0);
                    Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                    ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                    problem.AddResidualBlock(f, loss_function, para_Pose[start], relo_Pose, para_Ex_Pose[0], para_Feature[feature_index]);
                    retrive_feature_index++;
                }
            }
        }

    }

    ceres::Solver::Options options;
    //options.linear_solver_type = ceres::DENSE_SCHUR;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;

    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    ROS_DEBUG("solver costs: %f", t_solver.toc());

    double2vector();

    // the marginlization part

    {
        TicToc t_whole_marginalization;
        if (marginalization_flag == MARGIN_OLD)
        {
            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();

            if (last_marginalization_info)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                        last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            // IMU oldest frame prior info
            {
                if (pre_integrations[1]->sum_dt < 10.0)
                {
                    IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                                   vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                                   vector<int>{0, 1});
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                }
            }

            // point feature oldest prior info
            {
                int feature_index = -1;
                for (auto &it_per_id : f_manager.feature)
                {
                    it_per_id.used_num = it_per_id.feature_per_frame.size();
                    if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                        continue;

                    ++feature_index;

                    int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                    if (imu_i != 0)
                        continue;

                    Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                    for (auto &it_per_frame : it_per_id.feature_per_frame)
                    {
                        imu_j++;
                        if (imu_i == imu_j)
                            continue;

                        Vector3d pts_j = it_per_frame.point;
                        if (ESTIMATE_TD)
                        {
                            ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                              it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                                                                              it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                           vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{0, 3});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                        else
                        {
                            ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]},
                                                                                           vector<int>{0, 3});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            }

            if (0)
            {
                Eigen::Vector3d _Tic_(para_Ex_Pose[0][0], para_Ex_Pose[0][1], para_Ex_Pose[0][2]);
                Eigen::Matrix3d _Ric_ = Quaterniond(para_Ex_Pose[0][6],
                                                    para_Ex_Pose[0][3],
                                                    para_Ex_Pose[0][4],
                                                    para_Ex_Pose[0][5]).normalized().toRotationMatrix();

                lineMatchFilter.clear();
                lineMatchFilter = lineMatchesFilter[0];
                for (unsigned int i = 0; i < lineMatchFilter.size(); ++i)
                {
                    Vector3d line_param_marg(lineMatchFilter[i].detectedlineP.A,
                                             lineMatchFilter[i].detectedlineP.B, lineMatchFilter[i].detectedlineP.C);
                    const Eigen::Vector3d ptr_start = lineMatchFilter[i].line3dBodyP.PtrStart;
                    const Eigen::Vector3d ptr_end = lineMatchFilter[i].line3dBodyP.PtrEnd;
                    LineProjectionFactor *line_factor_marg = new LineProjectionFactor(ptr_start, ptr_end, line_param_marg,
                                                                                      K, _Ric_, _Tic_);
                    ResidualBlockInfo *residual_block_info_line = new ResidualBlockInfo(line_factor_marg, loss_function,
                                                                                        vector<double *>{para_Pose[0]},
                                                                                        vector<int>{0});
                    marginalization_info->addResidualBlockInfo(residual_block_info_line);
                }

            }


            TicToc t_pre_margin;
            marginalization_info->preMarginalize();
            ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());

            TicToc t_margin;
            marginalization_info->marginalize();
            ROS_DEBUG("marginalization %f ms", t_margin.toc());

            std::unordered_map<long, double *> addr_shift;
            for (int i = 1; i <= WINDOW_SIZE; i++)
            {
                addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
            if (ESTIMATE_TD)
            {
                addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
            }
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;    // add the margin info into the prior constraint
            last_marginalization_parameter_blocks = parameter_blocks;

        }
        else
        {
            if (last_marginalization_info &&
                std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
            {

                MarginalizationInfo *marginalization_info = new MarginalizationInfo();
                vector2double();
                if (last_marginalization_info)
                {
                    vector<int> drop_set;
                    for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                    {
                        ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                        if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                            drop_set.push_back(i);
                    }
                    // construct new marginlization_factor
                    MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                                   last_marginalization_parameter_blocks,
                                                                                   drop_set);

                    marginalization_info->addResidualBlockInfo(residual_block_info);
                }

                TicToc t_pre_margin;
                ROS_DEBUG("begin marginalization");
                marginalization_info->preMarginalize();
                ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

                TicToc t_margin;
                ROS_DEBUG("begin marginalization");
                marginalization_info->marginalize();
                ROS_DEBUG("end marginalization, %f ms", t_margin.toc());

                std::unordered_map<long, double *> addr_shift;
                for (int i = 0; i <= WINDOW_SIZE; i++)
                {
                    if (i == WINDOW_SIZE - 1)
                        continue;
                    else if (i == WINDOW_SIZE)
                    {
                        addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                    }
                    else
                    {
                        addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                    }
                }
                for (int i = 0; i < NUM_OF_CAM; i++)
                    addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
                if (ESTIMATE_TD)
                {
                    addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
                }

                vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
                if (last_marginalization_info)
                    delete last_marginalization_info;
                last_marginalization_info = marginalization_info;
                last_marginalization_parameter_blocks = parameter_blocks;

            }
        }
        ROS_DEBUG("whole marginalization costs: %f", t_whole_marginalization.toc());

        ROS_DEBUG("whole time for ceres: %f", t_whole.toc());
    }

}

void Estimator::slideWindowWithLinesFoV()
{
    TicToc t_Lmargin;
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0].stamp.toSec();
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Rs[i].swap(Rs[i + 1]);

                std::swap(pre_integrations[i], pre_integrations[i + 1]);

                dt_buf[i].swap(dt_buf[i + 1]);
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                Headers[i] = Headers[i + 1];
                Ps[i].swap(Ps[i + 1]);
                Vs[i].swap(Vs[i + 1]);
                Bas[i].swap(Bas[i + 1]);
                Bgs[i].swap(Bgs[i + 1]);

                // for the 3d line in FoV
                WorldLinesInFOV[i].swap(WorldLinesInFOV[i + 1]);
                images_show[i] = images_show[i + 1];

            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

            // for the 3d line in FoV
            WorldLinesInFOV[WINDOW_SIZE] = WorldLinesInFOV[WINDOW_SIZE - 1];

            images_show[WINDOW_SIZE] = images_show[WINDOW_SIZE - 1];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();


            if (true || solver_flag == INITIAL)    // point initialization failure
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                it_0->second.pre_integration = nullptr;

                for (map<double, ImageFrame>::iterator it = all_image_frame.begin(); it != it_0; ++it)
                {
                    if (it->second.pre_integration)
                        delete it->second.pre_integration;
                    it->second.pre_integration = NULL;
                }

                all_image_frame.erase(all_image_frame.begin(), it_0);
                all_image_frame.erase(t_0);

            }
            slideWindowOld();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {
            for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
            {
                double tmp_dt = dt_buf[frame_count][i];
                Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                dt_buf[frame_count - 1].push_back(tmp_dt);
                linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
            }

            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Vs[frame_count - 1] = Vs[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            Bas[frame_count - 1] = Bas[frame_count];
            Bgs[frame_count - 1] = Bgs[frame_count];

            // for 3d lines in Fov
            WorldLinesInFOV[frame_count - 1] = WorldLinesInFOV[frame_count];

            images_show[frame_count - 1] = images_show[frame_count];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            slideWindowNew();
        }
    }
}


// real marginalization is removed in solve_ceres()
void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}
// real marginalization is removed in solve_ceres()
void Estimator:: slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();
}

void Estimator::setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points, Vector3d _relo_t, Matrix3d _relo_r)
{
    relo_frame_stamp = _frame_stamp;
    relo_frame_index = _frame_index;
    match_points.clear();
    match_points = _match_points;
    prev_relo_t = _relo_t;
    prev_relo_r = _relo_r;
    for(int i = 0; i < WINDOW_SIZE; i++)
    {
        if(relo_frame_stamp == Headers[i].stamp.toSec())
        {
            relo_frame_local_index = i;
            relocalization_info = 1;
            for (int j = 0; j < SIZE_POSE; j++)
                relo_Pose[j] = para_Pose[i][j];
        }
    }
}

void Estimator::readBenchMark()
{
    std::string path_name = "/home/zx/Documents/Carla_Dataset/";

    std::string csv_file = path_name + "GT.csv";
    std::FILE *f = std::fopen(csv_file.c_str(), "r");
    if (f==NULL)
    {
        std::cout << "can't load ground truth; wrong path" << std::endl;
        return;
    }
    while (!std::feof(f))
    {
        benchmark.emplace_back(f);
    }
    std::fclose(f);
    benchmark.pop_back();
    ROS_INFO("Data loaded: %d", (int)benchmark.size());
}

int Estimator::getGTIndex(float time_stamp, int preIndex)
{
    for (unsigned int j = preIndex; j < benchmark.size(); ++j)
    {
        float time_now = benchmark[j].t;
        if (abs(time_stamp - time_now) < 0.0001)
            return j;
    }
}