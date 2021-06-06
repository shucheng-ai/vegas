#pragma once
#include <opencv2/opencv.hpp>
#include "vegas.h"

namespace vegas {

    // content related to raster image algorithms with opencv

    class ImageLog {
        string root;
    public:
        ImageLog (string const &r = ""): root(r) {
            if (root.size()) {
                system(fmt::format("mkdir -p \"{}\"", root).c_str());
            }
        }

        bool enabled () const {
            return root.size() > 0;
        }

        void log (cv::Mat image, string const &path) const {
            if (root.size()) {
                cv::imwrite(fmt::format("%s/%s", root, path), image);
            }
        }

        /*
        void log (cv::Mat image, boost::basic_format<char> const &path) const {
            if (root.size()) {
                cv::imwrite((format{"%s/%s"} % root % path.str()).str(), image);
            }
        }
        */
    };

    extern ImageLog ilog;

    class RasterCanvas {
        Box bb;
        int scale_num;
        double scale_denom;
        int pad;
    public:
        cv::Mat image;

        RasterCanvas (Box const &bb_, int size, int pad_ = 0, int ctype = CV_8U): bb(bb_), pad(pad_) {
            CHECK(!bb.empty());
            scale_num = size - 1 - 2 * pad;
            scale_denom = std::max(bb.width(), bb.height());
            int rows = int(std::round(bb.height() / scale_denom * scale_num)) + 1 + pad * 2;
            int cols = int(std::round(bb.width() / scale_denom * scale_num)) + 1 + pad * 2;
            image = cv::Mat(rows, cols, ctype, cv::Scalar::all(0));
        }

        double scale (int v) const {
            return v * scale_num / scale_denom;
        }

        double unscale (int v) const {
            return v * scale_denom / scale_num;
        }

        void map (Point *pt) const {
            pt->at(0) = scale(pt->at(0) - bb[0][0]) + pad;
            pt->at(1) = image.rows - pad - scale(pt->at(1) - bb[0][1]);
        }

        cv::Point map (Point const &pt) const {
            Point v = pt;
            map(&v);
            v = round(v);
            return cv::Point(v[0], v[1]);
        }

        cv::Rect map (Box const &rect) const {
            cv::Point a = map(rect[0]);
            cv::Point b = map(rect[1]);
            std::swap(a.y, b.y);
            return cv::Rect(a, b);
        }

        void unmap (Point *pt) const {
            pt->at(0) = bb[0][0] + unscale(pt->at(0) - pad);
            pt->at(1) = bb[0][1] + unscale(image.rows - pad - pt->at(1));

        }

        void draw (Box const &r, cv::Scalar const &color = cv::Scalar(1), int thickness= 1) {
            cv::rectangle(image, map(r), color, thickness);
        }

        void draw (Point const &pt, int r, cv::Scalar const &color = cv::Scalar(1), int thickness= 1) {
            cv::Point p = map(pt);
            cv::circle(image, p, r, color);
        }

        void draw (Line const &l, cv::Scalar const &color = cv::Scalar(1), int thickness = 1) {
            cv::Point p1 = map(l[0]);
            cv::Point p2 = map(l[1]);
            cv::line(image, p1, p2, color, thickness);
        }

        void draw (Layer const &layer, cv::Scalar const &color = cv::Scalar(1), int thickness=1) {
            for (auto const &l: layer.lines) {
                draw(l, color, thickness);
            }
        }

        void draw (Document const &cad, cv::Scalar const &color = cv::Scalar(1), int thickness = 1) {
            for (auto const &l: cad.layers) {
                draw(l, color, thickness);
            }
        }
    };

}

