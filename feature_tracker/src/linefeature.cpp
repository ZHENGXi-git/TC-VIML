
#include "linefeature.h"
#include "parameters.h"

int LineFeatureTracker::n_id_line = 0;

Vec4f inBorder(const Vec4f &line)
{
    const int BORDER = 10;
    int col = COL - BORDER;
    int row = ROW - BORDER;
    float s_x, s_y, e_x, e_y;
    bool start_flag = false, end_flag = false;
    s_x = line[0];
    s_y = line[1];
    e_x = line[2];
    e_y = line[3];

    if (BORDER <= s_x && s_x < col && BORDER <= s_y && s_y < row)
        start_flag = true;
    if (BORDER <= e_x && e_x < col && BORDER <= e_y && e_y < row)
        end_flag = true;
    if (start_flag && end_flag)
        return line;

    Vec4f new_line = line;
    float delta_x = 0.2 * abs(e_x - s_x);
    float delta_y = 0.2 * abs(e_y - s_y);

    if (s_x < BORDER)
    {
        if (s_y <= e_y)
        {
            new_line[0] = line[0] + delta_x;
            new_line[1] = line[1] + delta_y;
        }
        else
        {
            new_line[0] = line[0] + delta_x;
            new_line[1] = line[1] - delta_y;
        }
    }
    if (s_x >= col)
    {
        if (s_y <= e_y)
        {
            new_line[0] = line[0] - delta_x;
            new_line[1] = line[1] + delta_y;
        }
        else
        {
            new_line[0] = line[0] - delta_x;
            new_line[1] = line[1] - delta_y;
        }
    }
    if (e_x < BORDER)
    {
        if (e_y <= s_y)
        {
            new_line[2] = line[2] + delta_x;
            new_line[3] = line[3] + delta_y;
        }
        else
        {
            new_line[2] = line[2] + delta_x;
            new_line[3] = line[3] - delta_y;
        }
    }
    if (e_x >= col)
    {
        if (e_y <= s_y)
        {
            new_line[2] = line[2] - delta_x;
            new_line[3] = line[3] + delta_y;
        }
        else
        {
            new_line[2] = line[2] - delta_x;
            new_line[3] = line[3] - delta_y;
        }
    }

    if (s_y < BORDER)
    {
        if (s_x <= e_x)
        {
            new_line[0] = line[0] + delta_x;
            new_line[1] = line[1] + delta_y;
        }
        else
        {
            new_line[0] = line[0] - delta_x;
            new_line[1] = line[1] + delta_y;
        }
    }
    if (s_y >= row)
    {
        if (s_x <= e_x)
        {
            new_line[0] = line[0] + delta_x;
            new_line[1] = line[1] - delta_y;
        }
        else
        {
            new_line[0] = line[0] - delta_x;
            new_line[1] = line[1] - delta_y;
        }
    }

    if (e_y < BORDER)
    {
        if (e_x <= s_x)
        {
            new_line[2] = line[2] + delta_x;
            new_line[3] = line[3] + delta_y;
        }
        else
        {
            new_line[2] = line[2] - delta_x;
            new_line[3] = line[3] + delta_y;
        }
    }
    if (e_y >= row)
    {
        if (e_x <= s_x)
        {
            new_line[2] = line[2] + delta_x;
            new_line[3] = line[3] - delta_y;
        }
        else
        {
            new_line[2] = line[2] - delta_x;
            new_line[3] = line[3] - delta_y;
        }
    }
    return new_line;

}

void reduceVector2(vector<Vec4f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[2*i] && status[2*i+1])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector2(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[2*i] && status[2*i+1])
            v[j++] = v[i];
    v.resize(j);
}

//

LineFeatureTracker::LineFeatureTracker()
{
    sum_time = 0.0;
}

void LineFeatureTracker::setMast()
{
    mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    if(mask.empty())
        cout << "mask is empty " << endl;

    vector<pair<int, pair<Vec4f, int>>> cnt_lines_id;

    for (int i = 0; i < int(forw_lines.size()); ++i)
    {
        cnt_lines_id.push_back(make_pair(track_cnt[i], make_pair(forw_lines[i], ids[i])));
    }
    sort(cnt_lines_id.begin(), cnt_lines_id.end(), [](const pair<int, pair<Vec4f, int>> &a, const pair<int, pair<Vec4f, int>> &b)
    {
        return a.first > b.first;
    });

    forw_lines.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_lines_id)
    {
        cv::Point s, e;
        s.x = int(it.second.first[0] + 0.5);
        s.y = int(it.second.first[1] + 0.5);
        e.x = int(it.second.first[2] + 0.5);
        e.y = int(it.second.first[3] + 0.5);
        if (mask.at<uchar>(s) == 255 || mask.at<uchar>(e) == 255)
        {
            forw_lines.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::line(mask, s, e, 0, MIN_DIST);
        }
    }
}


void LineFeatureTracker::addLines(vector<Vec4f> &lines)
{
    for (auto &p : lines)
    {
        forw_lines.push_back(p); //
       // cur_lines.push_back(p);
        ids.push_back(n_id_line++);
        track_cnt.push_back(1);
    }
}


void LineFeatureTracker::findForwMatchLine()
{
    vector<Vec4f>::iterator it;

    for (size_t i = 0; i < forw_pts.size() / 2; i = i + 2)
    {
        if (status[i] && status[i + 1])
        {
            cv::Point2f sPtr = forw_pts[i];
            cv::Point2f ePtr = forw_pts[i + 1];
            float A = ePtr.y - sPtr.y;
            float B = sPtr.x - ePtr.x;
            float C = ePtr.x * sPtr.y - sPtr.x * ePtr.y;
            float A2B2 = sqrt(A * A + B * B);
            float min_dist = 10000.0;
            int index = 0;
            for (size_t j = 0; j < forw_detect_lines.size(); ++j)
            {
                float dist = abs(A * forw_detect_lines[j][0] + B * forw_detect_lines[j][0] + C) / A2B2 +
                             abs(A * forw_detect_lines[j][2] + B * forw_detect_lines[j][3] + C) / A2B2;
                if (min_dist > dist)
                {
                    min_dist = dist;
                    index = j;
                }
            }
            if (min_dist < 10)
            {
                Vec4f match_line = forw_detect_lines[index];
                Vec4f track_line = inBorder(match_line);
                forw_lines.push_back(track_line);
                it = forw_detect_lines.begin() + index;
                forw_detect_lines.erase(it); // delete the tracked lines from the forw_detect_lines.
            }
            else
            {
                status[i] = 0;
                status[i + 1] = 0;
            }
        }
    }
    remain_detect_lines = forw_detect_lines; // the remained lines in forw_detect_lines
}

void LineFeatureTracker::updateForwPts()
{
    forw_pts.clear();
    cv::Point2f sP, eP;
    for (size_t i = 0; i < forw_lines.size(); ++i)
    {
        Vec4f inLine = inBorder(forw_lines[i]);
        sP.x = inLine[0];
        sP.y = inLine[1];
        eP.x = inLine[2];
        eP.y = inLine[3];
        forw_pts.push_back(sP);
        forw_pts.push_back(eP);
    }
}

void LineFeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;
    if (EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;

    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
        prev_img = img;
    }


    float distance_threshold = 5.4142;
    double canny_th1 = 50.0;
    double canny_th2 = 50.0;
    int canny_aperture_size = 3;
    bool do_merge = true;

    Ptr<cv::ximgproc::FastLineDetector> fld = cv::ximgproc::createFastLineDetector(LengthThreshold, distance_threshold,
                                                                                   canny_th1, canny_th2, canny_aperture_size,
                                                                                   do_merge);

    forw_pts.clear();
    forw_lines.clear();
    forw_detect_lines.clear();
    remain_detect_lines.clear();
    fld->detect(forw_img, forw_detect_lines);

    if (cur_pts.size() > 0) // begin tracking
    {
        TicToc t_o;
        status.clear();
        vector<float> err;
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        // get the forw_lines;
        findForwMatchLine();

        // delete the unsuccessful tracking lines, ids, and track_cnt.
        reduceVector2(cur_lines, status);
        reduceVector2(ids, status);
        reduceVector2(track_cnt, status);

        for (auto &n : track_cnt)
        {
            n++;
        }
    }

    if (cur_pts.size() == 0) // initial the starting lines
    {
        ids.clear();
        track_cnt.clear();
        cur_lines.clear();
        cur_pts.clear();
        addLines(forw_detect_lines);
    }

    int n_max_cnt = (MAX_CNT - 50) - static_cast<int>(forw_lines.size());
    if (n_max_cnt > 0)
    {
        addLines(remain_detect_lines);
    }
    cur_img = forw_img;
    updateForwPts();
    cur_pts = forw_pts;
    cur_lines = forw_lines; // update cur_lines;

}
















