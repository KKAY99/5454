//
// Inputs
//
Inputs
{
	Mat source0;
}

//
// Variables
//
Outputs
{
	Mat resizeImageOutput;
	Mat cvCvtcolorOutput;
	Mat hsvThresholdOutput;
	Mat cvErodeOutput;
	Mat cvDilateOutput;
	Mat cvMedianblurOutput;
	ContoursReport findContoursOutput;
	ContoursReport filterContoursOutput;
}

//
// Steps
//

Step Resize_Image0
{
    Mat resizeImageInput = source0;
    Double resizeImageWidth = 320.0;
    Double resizeImageHeight = 240.0;
    InterpolationType resizeImageInterpolation = CUBIC;

    resizeImage(resizeImageInput, resizeImageWidth, resizeImageHeight, resizeImageInterpolation, resizeImageOutput);
}

Step CV_cvtColor0
{
    Mat cvCvtcolorSrc = resizeImageOutput;
    ColorConversionType cvCvtcolorCode = COLOR_RGB2HSV_FULL;

    cvCvtcolor(cvCvtcolorSrc, cvCvtcolorCode, cvCvtcolorOutput);
}

Step HSV_Threshold0
{
    Mat hsvThresholdInput = cvCvtcolorOutput;
    List hsvThresholdHue = [0.0, 31.535837153932743];
    List hsvThresholdSaturation = [166.6366820819944, 255.0];
    List hsvThresholdValue = [105.48561479119088, 255.0];

    hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);
}

Step CV_erode0
{
    Mat cvErodeSrc = hsvThresholdOutput;
    Mat cvErodeKernel;
    Point cvErodeAnchor = (-1, -1);
    Double cvErodeIterations = 3.0;
    BorderType cvErodeBordertype = BORDER_CONSTANT;
    Scalar cvErodeBordervalue = (-1);

    cvErode(cvErodeSrc, cvErodeKernel, cvErodeAnchor, cvErodeIterations, cvErodeBordertype, cvErodeBordervalue, cvErodeOutput);
}

Step CV_dilate0
{
    Mat cvDilateSrc = cvErodeOutput;
    Mat cvDilateKernel;
    Point cvDilateAnchor = (-1, -1);
    Double cvDilateIterations = 3.0;
    BorderType cvDilateBordertype = BORDER_CONSTANT;
    Scalar cvDilateBordervalue = (-1);

    cvDilate(cvDilateSrc, cvDilateKernel, cvDilateAnchor, cvDilateIterations, cvDilateBordertype, cvDilateBordervalue, cvDilateOutput);
}

Step CV_medianBlur0
{
    Mat cvMedianblurSrc = cvDilateOutput;
    Double cvMedianblurKsize = 11.0;

    cvMedianblur(cvMedianblurSrc, cvMedianblurKsize, cvMedianblurOutput);
}

Step Find_Contours0
{
    Mat findContoursInput = cvMedianblurOutput;
    Boolean findContoursExternalOnly = true;

    findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);
}

Step Filter_Contours0
{
    ContoursReport filterContoursContours = findContoursOutput;
    Double filterContoursMinArea = 100.0;
    Double filterContoursMinPerimeter = 0.0;
    Double filterContoursMinWidth = 15.0;
    Double filterContoursMaxWidth = 1000.0;
    Double filterContoursMinHeight = 15.0;
    Double filterContoursMaxHeight = 1000.0;
    List filterContoursSolidity = [85.13189691433803, 100];
    Double filterContoursMaxVertices = 1000000.0;
    Double filterContoursMinVertices = 10.0;
    Double filterContoursMinRatio = 0.3;
    Double filterContoursMaxRatio = 1.3;

    filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);
}

Step NTPublish_ContoursReport0
{
    ContoursReport ntpublishContoursreportData = filterContoursOutput;
    String ntpublishContoursreportName = "Blue_Ball_Contours";
    Boolean ntpublishContoursreportPublishArea = false;
    Boolean ntpublishContoursreportPublishCenterx = true;
    Boolean ntpublishContoursreportPublishCentery = true;
    Boolean ntpublishContoursreportPublishWidth = true;
    Boolean ntpublishContoursreportPublishHeight = true;
    Boolean ntpublishContoursreportPublishSolidity = false;

    ntpublishContoursreport(ntpublishContoursreportData, ntpublishContoursreportName, ntpublishContoursreportPublishArea, ntpublishContoursreportPublishCenterx, ntpublishContoursreportPublishCentery, ntpublishContoursreportPublishWidth, ntpublishContoursreportPublishHeight, ntpublishContoursreportPublishSolidity, );
}




