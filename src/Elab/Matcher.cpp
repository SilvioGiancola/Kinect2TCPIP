#include "Matcher.h"


bool Matcher::reciprok = true;
MatchingMethod Matcher::myMethod = CSHOT;
DescriptionMethod Matcher::myDescMethod = RadiusSearch;

double Matcher::radiusSearchValue = 0.05;
int Matcher::kSearchValue = 10;



using namespace std;
Matcher::Matcher()
{   

}


Matcher::~Matcher()
{

}



pcl::CorrespondencesPtr Matcher::match(PointCloudNormalT::Ptr Input, PointCloudNormalT::Ptr Input_KP, PointCloudNormalT::Ptr Target, PointCloudNormalT::Ptr Target_KP)
{
    pcl::ScopeTime t ("Estimate Correspondences");

    pcl::CorrespondencesPtr myCorrespondences(new pcl::Correspondences);

    if (myMethod == PFH)
    {
        pcl::PointCloud<pcl::PFHSignature125>::Ptr Input_desc(new pcl::PointCloud<pcl::PFHSignature125>);
        Input_desc = DescribePFH(Input, Input_KP);
        pcl::PointCloud<pcl::PFHSignature125>::Ptr Target_desc(new pcl::PointCloud<pcl::PFHSignature125>);
        Target_desc = DescribePFH(Target, Target_KP);

      // myCorrespondences = MatchDescriptor<pcl::PFHSignature125>(Input_desc, Target_desc);
        myCorrespondences = MatchPFH(Input_desc, Target_desc);
    }
    else if (myMethod == PFHRGB)
    {
        pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr Input_desc(new pcl::PointCloud<pcl::PFHRGBSignature250>);
        Input_desc = DescribePFHRGB(Input, Input_KP);
        pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr Target_desc(new pcl::PointCloud<pcl::PFHRGBSignature250>);
        Target_desc = DescribePFHRGB(Target, Target_KP);

        myCorrespondences = MatchPFHRGB(Input_desc, Target_desc);
    }
    else if ( myMethod == FPFH)
    {
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr Input_desc(new pcl::PointCloud<pcl::FPFHSignature33>);
        Input_desc = DescribeFPFH(Input, Input_KP);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr Target_desc(new pcl::PointCloud<pcl::FPFHSignature33>);
        Target_desc = DescribeFPFH(Target, Target_KP);

        myCorrespondences = MatchFPFH(Input_desc, Target_desc);
    }
    else if (myMethod == SHOT)
    {
        pcl::PointCloud<pcl::SHOT352>::Ptr Input_desc(new pcl::PointCloud<pcl::SHOT352>);
        Input_desc = DescribeSHOT(Input, Input_KP);
        pcl::PointCloud<pcl::SHOT352>::Ptr Target_desc(new pcl::PointCloud<pcl::SHOT352>);
        Target_desc = DescribeSHOT(Target, Target_KP);

        myCorrespondences = MatchSHOT(Input_desc, Target_desc);
    }
    else if (myMethod == CSHOT)
    {
        pcl::PointCloud<pcl::SHOT1344>::Ptr Input_desc(new pcl::PointCloud<pcl::SHOT1344>);
        Input_desc = DescribeCSHOT(Input, Input_KP);
        pcl::PointCloud<pcl::SHOT1344>::Ptr Target_desc(new pcl::PointCloud<pcl::SHOT1344>);
        Target_desc = DescribeCSHOT(Target, Target_KP);

        myCorrespondences = MatchCSHOT(Input_desc, Target_desc);
    }
    else if (myMethod == Dist)
    {
        myCorrespondences = MatchDist(Input_KP, Target_KP);

    }


    /*   Output->sensor_orientation_ = Input->sensor_orientation_;
    Output->sensor_origin_ = Input->sensor_origin_;*/
    qDebug() << "I have found " << myCorrespondences->size() << " Correspondences";

    return myCorrespondences;
}

//********************//
//***  DESCRIPTOR  ***//
//********************//
pcl::PointCloud<pcl::PFHSignature125>::Ptr Matcher::DescribePFH(PointCloudNormalT::Ptr input, PointCloudNormalT::Ptr keypoints)
{
    QString name;
    if (myDescMethod == RadiusSearch) name = QString("%1 (radius Search = %2)").arg(__FUNCTION__).arg(radiusSearchValue);
    else if (myDescMethod == KSearch) name = QString("%1 (K Search = %2)").arg(__FUNCTION__).arg(kSearchValue);
    pcl::ScopeTime t (name.toStdString().c_str());

    pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptor (new pcl::PointCloud<pcl::PFHSignature125>);
    cout << "Description with PFH..." <<endl;

    QElapsedTimer *Timer = new QElapsedTimer();
    Timer->start();


    // Calculate features PFH
    pcl::PFHEstimation<PointNormalT, PointNormalT, pcl::PFHSignature125> pfh;
    pfh.setInputCloud (keypoints);
    pfh.setSearchSurface(input);
    pfh.setInputNormals (input);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointNormalT>::Ptr tree (new pcl::search::KdTree<PointNormalT>);

    pfh.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    if (myDescMethod == RadiusSearch)
        pfh.setRadiusSearch (radiusSearchValue);
    else if (myDescMethod == KSearch)
        pfh.setKSearch(kSearchValue);

    pfh.compute(*descriptor);
    // Compute the features

    cout << "OK. keypoints described in " << Timer->elapsed() << " milliseconds" << endl;

    /*
    // Check if there are NAN points in input cloud
    for ( size_t i=0; i<input_noNAN->size(); i++ )
    {
        cout <<  "x:   " << input_noNAN->points[i].x << "   y:   " << input_noNAN->points[i].y << "   z:   " << input_noNAN->points[i].z <<"  normal_x:   " << input_noNAN->points[i].normal_x  << "  normal_y:   "<< input_noNAN->points[i].normal_y <<"  normal_z:   "<< input_noNAN->points[i].normal_z <<endl;
    }

    // Visualize histogram
    for ( size_t i=0; i<descriptor->size(); i++ )
        for ( int j=0; j<33; j++ )
            cout << descriptor->points[i].histogram[j] <<endl;
*/

    pcl::PointIndices::Ptr indices_NAN_feature (new pcl::PointIndices);
    bool found_NAN_at_i =false;

    //Take Nan indices of point of features that have an histogram value NAN
    for ( size_t i=0; i<descriptor->size(); i++ )
    {
        found_NAN_at_i =false;
        for ( int j=0; j<125; j++ )
        {
            if( !pcl_isfinite(descriptor->points[i].histogram[j]))
            {
                found_NAN_at_i =true;
            }
        }
        if(found_NAN_at_i)
            indices_NAN_feature->indices.push_back(i);
    }

    // Filter descriptor with the indices founded
    pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptor_noNAN ( new pcl::PointCloud<pcl::PFHSignature125>);

    pcl::ExtractIndices<pcl::PFHSignature125> extract;
    extract.setInputCloud (descriptor);
    extract.setIndices (indices_NAN_feature);
    extract.setNegative (true);
    extract.filter (*descriptor_noNAN);
    *descriptor = *descriptor_noNAN;

    /*
      // Check same length -> obviously not!
      cout << "Try: Is:" << endl;
      cout << descriptor->size() << " = " << keypoints->size() << endl;
     */

    // We have to erase the keypoints that have no more features.
    pcl::ExtractIndices<pcl::PointXYZRGBNormal> extractKP;
    extractKP.setInputCloud (keypoints);
    extractKP.setIndices (indices_NAN_feature);
    extractKP.setNegative (true);
    extractKP.filter (*keypoints);

    // Check same length ->obviously yes!
  //  cout << "Try2: Is:" << endl;
  //  cout <<  descriptor->size() << " = " << keypoints->size() << endl;
    return descriptor;
}

pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr Matcher::DescribePFHRGB(PointCloudNormalT::Ptr input,PointCloudNormalT::Ptr keypoints)
{
    QString name;
    if (myDescMethod == RadiusSearch) name = QString("%1 (radius Search = %2)").arg(__FUNCTION__).arg(radiusSearchValue);
    else if (myDescMethod == KSearch) name = QString("%1 (K Search = %2)").arg(__FUNCTION__).arg(kSearchValue);
    pcl::ScopeTime t (name.toStdString().c_str());


    pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr descriptor(new pcl::PointCloud<pcl::PFHRGBSignature250>);
    std::cout << "Description with PFHRGB..." <<endl;

    QElapsedTimer *Timer = new QElapsedTimer();
    Timer->start();


    // Calculate features PFHRGB
    pcl::PFHRGBEstimation<PointNormalT, PointNormalT, pcl::PFHRGBSignature250> pfhrgb;
    pfhrgb.setInputCloud (keypoints);
    pfhrgb.setSearchSurface(input);
    pfhrgb.setInputNormals (input);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointNormalT>::Ptr tree (new pcl::search::KdTree<PointNormalT>);

    pfhrgb.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  //  pfhrgb.setRadiusSearch ();
    if (myDescMethod == RadiusSearch)
        pfhrgb.setRadiusSearch (radiusSearchValue);
    else if (myDescMethod == KSearch)
        pfhrgb.setKSearch(kSearchValue);

    // Compute the features
    pfhrgb.compute(*descriptor);

    cout << "OK. keypoints described in " << Timer->elapsed() << " milliseconds" << endl;

    /*
    // Check if there are NAN points in input cloud
    for ( size_t i=0; i<input_noNAN->size(); i++ )
    {
        cout <<  "x:   " << input_noNAN->points[i].x << "   y:   " << input_noNAN->points[i].y << "   z:   " << input_noNAN->points[i].z <<"  normal_x:   " << input_noNAN->points[i].normal_x  << "  normal_y:   "<< input_noNAN->points[i].normal_y <<"  normal_z:   "<< input_noNAN->points[i].normal_z <<endl;
    }

    // Visualize histogram
    for ( size_t i=0; i<descriptor->size(); i++ )
        for ( int j=0; j<33; j++ )
            cout << descriptor->points[i].histogram[j] <<endl;
*/

    pcl::PointIndices::Ptr indices_NAN_feature (new pcl::PointIndices);
    bool found_NAN_at_i =false;

    //Take Nan indices of point of features that have an histogram value NAN
    for ( size_t i=0; i<descriptor->size(); i++ )
    {
        found_NAN_at_i =false;
        for ( int j=0; j<250; j++ )
        {
            if( !pcl_isfinite(descriptor->points[i].histogram[j]))
            {
                found_NAN_at_i =true;
            }
        }
        if(found_NAN_at_i)
            indices_NAN_feature->indices.push_back(i);
    }

    // Filter descriptor with the indices founded
    pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr descriptor_noNAN ( new pcl::PointCloud<pcl::PFHRGBSignature250>);

    pcl::ExtractIndices<pcl::PFHRGBSignature250> extract;
    extract.setInputCloud (descriptor);
    extract.setIndices (indices_NAN_feature);
    extract.setNegative (true);
    extract.filter (*descriptor_noNAN);
    *descriptor = *descriptor_noNAN;

    /*
      // Check same length -> obviously not!
      cout << "Try: Is:" << endl;
      cout << descriptor->size() << " = " << keypoints->size() << endl;
     */

    // We have to erase the keypoints that have no more features.
    pcl::ExtractIndices<pcl::PointXYZRGBNormal> extractKP;
    extractKP.setInputCloud (keypoints);
    extractKP.setIndices (indices_NAN_feature);
    extractKP.setNegative (true);
    extractKP.filter (*keypoints);

    // Check same length ->obviously yes!
  //  cout << "Try2: Is:" << endl;
 //   cout <<  descriptor->size() << " = " << keypoints->size() << endl;
    return descriptor;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr Matcher::DescribeFPFH(PointCloudNormalT::Ptr input, PointCloudNormalT::Ptr keypoints)
{
    QString name;
    if (myDescMethod == RadiusSearch) name = QString("%1 (radius Search = %2)").arg(__FUNCTION__).arg(radiusSearchValue);
    else if (myDescMethod == KSearch) name = QString("%1 (K Search = %2)").arg(__FUNCTION__).arg(kSearchValue);
    pcl::ScopeTime t (name.toStdString().c_str());

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptor(new pcl::PointCloud<pcl::FPFHSignature33>);
    std::cout << "Description with FPFH..." <<std::endl;

    QElapsedTimer *Timer = new QElapsedTimer();
    Timer->start();


    // Calculate features FPFH
    pcl::FPFHEstimation<PointNormalT, PointNormalT, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud (keypoints);
    fpfh.setSearchSurface(input);
    fpfh.setInputNormals (input);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointNormalT>::Ptr tree (new pcl::search::KdTree<PointNormalT>);

    fpfh.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  //  fpfh.setRadiusSearch (radiusSearchValue);
    if (myDescMethod == RadiusSearch)
        fpfh.setRadiusSearch (radiusSearchValue);
    else if (myDescMethod == KSearch)
        fpfh.setKSearch(kSearchValue);

    // Compute the features
    fpfh.compute(*descriptor);

    cout << "OK. keypoints described in " << Timer->elapsed() << " milliseconds" << endl;

    /*
    // Check if there are NAN points in input cloud
    for ( size_t i=0; i<input_noNAN->size(); i++ )
    {
        cout <<  "x:   " << input_noNAN->points[i].x << "   y:   " << input_noNAN->points[i].y << "   z:   " << input_noNAN->points[i].z <<"  normal_x:   " << input_noNAN->points[i].normal_x  << "  normal_y:   "<< input_noNAN->points[i].normal_y <<"  normal_z:   "<< input_noNAN->points[i].normal_z <<endl;
    }

    // Visualize histogram
    for ( size_t i=0; i<descriptor->size(); i++ )
        for ( int j=0; j<33; j++ )
            cout << descriptor->points[i].histogram[j] <<endl;
*/

    pcl::PointIndices::Ptr indices_NAN_feature (new pcl::PointIndices);
    bool found_NAN_at_i =false;

    //Take Nan indices of point of features that have an histogram value NAN
    for ( size_t i=0; i<descriptor->size(); i++ )
    {
        found_NAN_at_i =false;
        for ( int j=0; j<33; j++ )
        {
            if( !pcl_isfinite(descriptor->points[i].histogram[j]))
            {
                found_NAN_at_i =true;
            }
        }
        if(found_NAN_at_i)
            indices_NAN_feature->indices.push_back(i);
    }

    // Filter descriptor with the indices founded
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptor_noNAN ( new pcl::PointCloud<pcl::FPFHSignature33>);

    pcl::ExtractIndices<pcl::FPFHSignature33> extract;
    extract.setInputCloud (descriptor);
    extract.setIndices (indices_NAN_feature);
    extract.setNegative (true);
    extract.filter (*descriptor_noNAN);
    *descriptor = *descriptor_noNAN;

    /*
      // Check same length -> obviously not!
      cout << "Try: Is:" << endl;
      cout << descriptor->size() << " = " << keypoints->size() << endl;
     */

    // We have to erase the keypoints that have no more features.
    pcl::ExtractIndices<pcl::PointXYZRGBNormal> extractKP;
    extractKP.setInputCloud (keypoints);
    extractKP.setIndices (indices_NAN_feature);
    extractKP.setNegative (true);
    extractKP.filter (*keypoints);


    // Check same length ->obviously yes!
 //   cout << "Try2: Is:" << endl;
  //  cout <<  descriptor->size() << " = " << keypoints->size() << endl;


    return descriptor;
}

pcl::PointCloud<pcl::SHOT352>::Ptr Matcher::DescribeSHOT(PointCloudNormalT::Ptr input,PointCloudNormalT::Ptr keypoints)
{

    QString name;
    if (myDescMethod == RadiusSearch) name = QString("%1 (radius Search = %2)").arg(__FUNCTION__).arg(radiusSearchValue);
    else if (myDescMethod == KSearch) name = QString("%1 (K Search = %2)").arg(__FUNCTION__).arg(kSearchValue);
    pcl::ScopeTime t (name.toStdString().c_str());


    pcl::PointCloud<pcl::SHOT352>::Ptr descriptor(new pcl::PointCloud<pcl::SHOT352>);
    cout << "Description with SHOT..." <<endl;

    QElapsedTimer *Timer = new QElapsedTimer();
    Timer->start();


    // Calculate features SHOT
    pcl::SHOTEstimation<PointNormalT, PointNormalT, pcl::SHOT352> shot;
    shot.setInputCloud (keypoints);
    shot.setSearchSurface(input);
    shot.setInputNormals (input);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointNormalT>::Ptr tree (new pcl::search::KdTree<PointNormalT>);

    shot.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
   // shot.setRadiusSearch (radiusSearchValue);
    if (myDescMethod == RadiusSearch)
        shot.setRadiusSearch (radiusSearchValue);
    else if (myDescMethod == KSearch)
        shot.setKSearch(kSearchValue);

    // Compute the features
    shot.compute(*descriptor);

    cout << "OK. keypoints described in " << Timer->elapsed() << " milliseconds" << endl;

    /*
    // Check if there are NAN points in input cloud
    for ( size_t i=0; i<input_noNAN->size(); i++ )
    {
        cout <<  "x:   " << input_noNAN->points[i].x << "   y:   " << input_noNAN->points[i].y << "   z:   " << input_noNAN->points[i].z <<"  normal_x:   " << input_noNAN->points[i].normal_x  << "  normal_y:   "<< input_noNAN->points[i].normal_y <<"  normal_z:   "<< input_noNAN->points[i].normal_z <<endl;
    }

    // Visualize histogram
    for ( size_t i=0; i<descriptor->size(); i++ )
        for ( int j=0; j<33; j++ )
            cout << descriptor->points[i].histogram[j] <<endl;
*/

    pcl::PointIndices::Ptr indices_NAN_feature (new pcl::PointIndices);
    bool found_NAN_at_i =false;

    //Take Nan indices of point of features that have an histogram value NAN
    for ( size_t i=0; i<descriptor->size(); i++ )
    {
        found_NAN_at_i =false;
        for ( int j=0; j<352; j++ )
        {
            if( !pcl_isfinite(descriptor->points[i].descriptor[j]))
            {
                found_NAN_at_i =true;
            }
        }
        if(found_NAN_at_i)
            indices_NAN_feature->indices.push_back(i);
    }

    // Filter descriptor with the indices founded
    pcl::PointCloud<pcl::SHOT352>::Ptr descriptor_noNAN ( new pcl::PointCloud<pcl::SHOT352>);

    pcl::ExtractIndices<pcl::SHOT352> extract;
    extract.setInputCloud (descriptor);
    extract.setIndices (indices_NAN_feature);
    extract.setNegative (true);
    extract.filter (*descriptor_noNAN);
    *descriptor = *descriptor_noNAN;

    /*
      // Check same length -> obviously not!
      cout << "Try: Is:" << endl;
      cout << descriptor->size() << " = " << keypoints->size() << endl;
     */

    // We have to erase the keypoints that have no more features.
    pcl::ExtractIndices<pcl::PointXYZRGBNormal> extractKP;
    extractKP.setInputCloud (keypoints);
    extractKP.setIndices (indices_NAN_feature);
    extractKP.setNegative (true);
    extractKP.filter (*keypoints);

    // Check same length ->obviously yes!
 //   cout << "Try2: Is:" << endl;
 //   cout <<  descriptor->size() << " = " << keypoints->size() << endl;
    return descriptor;
}

pcl::PointCloud<pcl::SHOT1344>::Ptr Matcher::DescribeCSHOT(PointCloudNormalT::Ptr input,PointCloudNormalT::Ptr keypoints )
{
    QString name;
    if (myDescMethod == RadiusSearch) name = QString("%1 (radius Search = %2)").arg(__FUNCTION__).arg(radiusSearchValue);
    else if (myDescMethod == KSearch) name = QString("%1 (K Search = %2)").arg(__FUNCTION__).arg(kSearchValue);
    pcl::ScopeTime t (name.toStdString().c_str());

    pcl::PointCloud<pcl::SHOT1344>::Ptr descriptor(new pcl::PointCloud<pcl::SHOT1344>);

    cout << "Description with SHOTRGB..." <<endl;

    QElapsedTimer *Timer = new QElapsedTimer();
    Timer->start();


    // Calculate features FPFH
    pcl::SHOTColorEstimation<PointNormalT, PointNormalT, pcl::SHOT1344> shotrgb;
    shotrgb.setInputCloud (keypoints);
    shotrgb.setSearchSurface(input);
    shotrgb.setInputNormals (input);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointNormalT>::Ptr tree (new pcl::search::KdTree<PointNormalT>);

    shotrgb.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    // shotrgb.setRadiusSearch (radiusSearchValue);
    if (myDescMethod == RadiusSearch)
        shotrgb.setRadiusSearch (radiusSearchValue);
    else if (myDescMethod == KSearch)
        shotrgb.setKSearch(kSearchValue);

    // Compute the features
    shotrgb.compute(*descriptor);

    cout << "OK. keypoints described in " << Timer->elapsed() << " milliseconds" << endl;

    /*
    // Check if there are NAN points in input cloud
    for ( size_t i=0; i<input_noNAN->size(); i++ )
    {
        cout <<  "x:   " << input_noNAN->points[i].x << "   y:   " << input_noNAN->points[i].y << "   z:   " << input_noNAN->points[i].z <<"  normal_x:   " << input_noNAN->points[i].normal_x  << "  normal_y:   "<< input_noNAN->points[i].normal_y <<"  normal_z:   "<< input_noNAN->points[i].normal_z <<endl;
    }

    // Visualize histogram
    for ( size_t i=0; i<descriptor->size(); i++ )
        for ( int j=0; j<33; j++ )
            cout << descriptor->points[i].histogram[j] <<endl;
*/

    pcl::PointIndices::Ptr indices_NAN_feature (new pcl::PointIndices);
    bool found_NAN_at_i =false;

    //Take Nan indices of point of features that have an histogram value NAN
    for ( size_t i=0; i<descriptor->size(); i++ )
    {
        found_NAN_at_i =false;
        for ( int j=0; j<1344; j++ )
            if( !pcl_isfinite(descriptor->points[i].descriptor[j]))
                found_NAN_at_i =true;


        if(found_NAN_at_i)
            indices_NAN_feature->indices.push_back(i);
    }

    // Filter descriptor with the indices founded
    pcl::PointCloud<pcl::SHOT1344>::Ptr descriptor_noNAN ( new pcl::PointCloud<pcl::SHOT1344>);

    pcl::ExtractIndices<pcl::SHOT1344> extract;
    extract.setInputCloud (descriptor);
    extract.setIndices (indices_NAN_feature);
    extract.setNegative (true);
    extract.filter (*descriptor_noNAN);
    *descriptor = *descriptor_noNAN;

    /*
      // Check same length -> obviously not!
      cout << "Try: Is:" << endl;
      cout << descriptor->size() << " = " << keypoints->size() << endl;
     */

    // We have to erase the keypoints that have no more features.
    pcl::ExtractIndices<pcl::PointXYZRGBNormal> extractKP;
    extractKP.setInputCloud (keypoints);
    extractKP.setIndices (indices_NAN_feature);
    extractKP.setNegative (true);
    extractKP.filter (*keypoints);

    // Check same length ->obviously yes!
 //   cout << "Try2: Is:" << endl;
  //  cout <<  descriptor->size() << " = " << keypoints->size() << endl;
    return descriptor;
}

//****************//
//*** MATCHING ***//
//****************//

pcl::CorrespondencesPtr Matcher::MatchPFH(pcl::PointCloud<pcl::PFHSignature125>::Ptr Input_desc, pcl::PointCloud<pcl::PFHSignature125>::Ptr Target_desc)
{
    pcl::CorrespondencesPtr myCorrespondences(new pcl::Correspondences);
    pcl::registration::CorrespondenceEstimationBase<pcl::PFHSignature125, pcl::PFHSignature125>::Ptr myCorrespondenceEstimation;
    myCorrespondenceEstimation.reset(new pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125, pcl::PFHSignature125>);
    //  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    myCorrespondenceEstimation->setInputSource (Input_desc);
    myCorrespondenceEstimation->setInputTarget (Target_desc);
    if (reciprok)
        myCorrespondenceEstimation->determineReciprocalCorrespondences (*myCorrespondences);
    else
        myCorrespondenceEstimation->determineCorrespondences (*myCorrespondences);

    return myCorrespondences;
}

pcl::CorrespondencesPtr Matcher::MatchPFHRGB(pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr Input_desc, pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr Target_desc)
{
    pcl::CorrespondencesPtr myCorrespondences(new pcl::Correspondences);
    pcl::registration::CorrespondenceEstimationBase<pcl::PFHRGBSignature250, pcl::PFHRGBSignature250>::Ptr myCorrespondenceEstimation;
    myCorrespondenceEstimation.reset(new pcl::registration::CorrespondenceEstimation<pcl::PFHRGBSignature250, pcl::PFHRGBSignature250>);
    //  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    myCorrespondenceEstimation->setInputSource (Input_desc);
    myCorrespondenceEstimation->setInputTarget (Target_desc);
    if (reciprok)
        myCorrespondenceEstimation->determineReciprocalCorrespondences (*myCorrespondences);
    else
        myCorrespondenceEstimation->determineCorrespondences (*myCorrespondences);

    return myCorrespondences;
}

pcl::CorrespondencesPtr Matcher::MatchFPFH(pcl::PointCloud<pcl::FPFHSignature33>::Ptr Input_desc, pcl::PointCloud<pcl::FPFHSignature33>::Ptr Target_desc)
{
    pcl::CorrespondencesPtr myCorrespondences(new pcl::Correspondences);
    pcl::registration::CorrespondenceEstimationBase<pcl::FPFHSignature33, pcl::FPFHSignature33>::Ptr myCorrespondenceEstimation;
    myCorrespondenceEstimation.reset(new pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33>);
    //  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    myCorrespondenceEstimation->setInputSource (Input_desc);
    myCorrespondenceEstimation->setInputTarget (Target_desc);
    if (reciprok)
        myCorrespondenceEstimation->determineReciprocalCorrespondences (*myCorrespondences);
    else
        myCorrespondenceEstimation->determineCorrespondences (*myCorrespondences);

    return myCorrespondences;
}

pcl::CorrespondencesPtr Matcher::MatchSHOT(pcl::PointCloud<pcl::SHOT352>::Ptr Input_desc, pcl::PointCloud<pcl::SHOT352>::Ptr Target_desc)
{
    pcl::CorrespondencesPtr myCorrespondences(new pcl::Correspondences);
    pcl::registration::CorrespondenceEstimationBase<pcl::SHOT352, pcl::SHOT352>::Ptr myCorrespondenceEstimation;
    myCorrespondenceEstimation.reset(new pcl::registration::CorrespondenceEstimation<pcl::SHOT352, pcl::SHOT352>);
    //  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    myCorrespondenceEstimation->setInputSource (Input_desc);
    myCorrespondenceEstimation->setInputTarget (Target_desc);
    if (reciprok)
        myCorrespondenceEstimation->determineReciprocalCorrespondences (*myCorrespondences);
    else
        myCorrespondenceEstimation->determineCorrespondences (*myCorrespondences);

    return myCorrespondences;
}

pcl::CorrespondencesPtr Matcher::MatchCSHOT(pcl::PointCloud<pcl::SHOT1344>::Ptr Input_desc, pcl::PointCloud<pcl::SHOT1344>::Ptr Target_desc)
{
    pcl::CorrespondencesPtr myCorrespondences(new pcl::Correspondences);
    pcl::registration::CorrespondenceEstimationBase<pcl::SHOT1344, pcl::SHOT1344>::Ptr myCorrespondenceEstimation;
    myCorrespondenceEstimation.reset(new pcl::registration::CorrespondenceEstimation<pcl::SHOT1344, pcl::SHOT1344>);
    //  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    myCorrespondenceEstimation->setInputSource (Input_desc);
    myCorrespondenceEstimation->setInputTarget (Target_desc);
    if (reciprok)
        myCorrespondenceEstimation->determineReciprocalCorrespondences (*myCorrespondences);
    else
        myCorrespondenceEstimation->determineCorrespondences (*myCorrespondences);

    return myCorrespondences;
}

pcl::CorrespondencesPtr Matcher::MatchDist(PointCloudNormalT::Ptr Input_desc, PointCloudNormalT::Ptr Target_desc)
{
    pcl::CorrespondencesPtr myCorrespondences(new pcl::Correspondences);
    pcl::registration::CorrespondenceEstimationBase<PointNormalT, PointNormalT>::Ptr myCorrespondenceEstimation;
    myCorrespondenceEstimation.reset(new pcl::registration::CorrespondenceEstimation<PointNormalT, PointNormalT>);
    //  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    myCorrespondenceEstimation->setInputSource (Input_desc);
    myCorrespondenceEstimation->setInputTarget (Target_desc);
    if (reciprok)
        myCorrespondenceEstimation->determineReciprocalCorrespondences (*myCorrespondences);
    else
        myCorrespondenceEstimation->determineCorrespondences (*myCorrespondences);

    return myCorrespondences;
}

/*
//template <typename Descriptor>
pcl::CorrespondencesPtr Matcher::MatchDescriptor(pcl::PointCloud<Descriptor>::Ptr Input_desc, pcl::PointCloud<Descriptor>::Ptr Target_desc)
{
    pcl::CorrespondencesPtr myCorrespondences(new pcl::Correspondences);
    pcl::registration::CorrespondenceEstimationBase<Descriptor, Descriptor>::Ptr myCorrespondenceEstimation;
    myCorrespondenceEstimation.reset(new pcl::registration::CorrespondenceEstimation<Descriptor, Descriptor>);
    //  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    myCorrespondenceEstimation->setInputSource (Input_desc);
    myCorrespondenceEstimation->setInputTarget (Target_desc);
    if (reciprok)
        myCorrespondenceEstimation->determineReciprocalCorrespondences (*myCorrespondences);
    else
        myCorrespondenceEstimation->determineCorrespondences (*myCorrespondences);

    return myCorrespondences;
}
*/
