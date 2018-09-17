    boost::multi_array<int, 2> matrix(boost::extents[n_N+2][n_E+2]);
    // make field larger to put in matrix (-2) on the borders for easier algorithm later
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

