polygon_t boostFieldBorderPolygon;
    for (auto point : fieldCoverageInfo.field.boundary_path) {
        geographic_msgs::GeoPoint geoPoint = geodesy::toMsg(point.latitude, point.longitude);
        latitudes.push_back(point.latitude);
        longitudes.push_back(point.longitude);
        geodesy::UTMPoint utmPoint((geoPoint));
        boost::geometry::append(boostFieldBorderPolygon.outer(), point_t(utmPoint.northing, utmPoint.easting));
    }

    double latitude_min = *std::min_element(latitudes.begin(),latitudes.end());
    double latitude_max = *std::max_element(latitudes.begin(),latitudes.end());
    double longitude_min = *std::min_element(longitudes.begin(),longitudes.end());
    double longitude_max = *std::max_element(longitudes.begin(),longitudes.end());
    geodesy::UTMPoint bottomLeft(geodesy::toMsg(latitude_min, longitude_min));
    geodesy::UTMPoint bottomRight(geodesy::toMsg(latitude_min, longitude_max));
    geodesy::UTMPoint topLeft(geodesy::toMsg(latitude_max, longitude_min));
    geodesy::UTMPoint topRight(geodesy::toMsg(latitude_max, longitude_max));

    // due to distortion they are NOT equal, even though we are coming from the same longitudine
    double leftBorder_E = std::min(bottomLeft.easting, topRight.easting);
    double bottomBorder_N = std::min(bottomLeft.northing, bottomRight.northing);
    double rightBorder_E = std::max(bottomRight.easting, topRight.easting);
    double topBorder_N = std::max(topLeft.northing, topRight.northing);
    double width = rightBorder_E - leftBorder_E;
    double height = topBorder_N - bottomBorder_N;

    float minDimFootprint = std::min(fieldCoverageInfo.thermal_camera_ground_footprint_width, fieldCoverageInfo.thermal_camera_ground_footprint_height);
    int n_E = std::ceil(width/minDimFootprint);
    int n_N = std::ceil(height/minDimFootprint);

    boost::multi_array<int, 2> matrix(boost::extents[n_N+2][n_E+2]);
    // make field larger to put in matrix (-2) on the borders for easier algorithm later
    bottomBorder_N -= minDimFootprint;
    leftBorder_E -= minDimFootprint;

    for (int i = 0; i < n_N+2; ++i) {
        for (int j = 0; j < n_E+2; ++j) {
            if (i == 0 || j == 0 || i == n_N + 1 || j == n_E + 1) {
                matrix[i][j] = -2;
                continue;
            }
            polygon_t cellBorderPolygon = helperFunctionGetSquarePolygonFromMatrixCell(bottomBorder_N, leftBorder_E, i, j, minDimFootprint);
            std::deque<polygon_t> output;
            boost::geometry::intersection(boostFieldBorderPolygon, cellBorderPolygon, output);
            if (output.size() > 0) {
                // intersection found
                matrix[i][j] = -1;
            } else {
                // not intersecting
                matrix[i][j] = -2;
s            }
        }
    }

