polygon_t boostFieldBorderPolygon;
    for (auto point : fieldCoverageInfo.field.boundary_path) {
        geographic_msgs::GeoPoint geoPoint = geodesy::toMsg(point.latitude, point.longitude);
        geodesy::UTMPoint utmPoint((geoPoint));
        boost::geometry::append(boostFieldBorderPolygon.outer(), point_t(utmPoint.northing, utmPoint.easting));
    }

