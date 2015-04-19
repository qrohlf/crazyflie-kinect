//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.DepthBasics
{
    using System;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using Emgu.CV;
    using Emgu.CV.Structure;
    using Emgu.CV.CvEnum;
    using System.Text;
    using ZeroMQ;
    using System.Timers;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Map depth range to byte range
        /// </summary>
        private const int MapDepthToByte = 8000 / 256;
        
        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Reader for depth frames
        /// </summary>
        private DepthFrameReader depthFrameReader = null;

        /// <summary>
        /// Description of the data contained in the depth frame
        /// </summary>
        private FrameDescription depthFrameDescription = null;
            
        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap depthBitmapRaw = null;

        private WriteableBitmap openCVBitmap = null;

        /// <summary>
        /// Intermediate storage for frame data converted to color
        /// </summary>
        private byte[] depthPixels = null;

        private byte[] monochromePixels = null;

        private byte[] colorPixels;

        // The image we're using for OpenCV tracking
        private Image<Gray, byte> depthImage;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        private double thrust = 0.5;

        private ZContext ZMQcontext = null;
        private ZSocket ZMQSocket = null;

        private int blobCount = 0;
        private int blob0_y = 300;

        private Timer PIDTimer;
        private int PIDTickMS = 50;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

            // open the reader for the depth frames
            this.depthFrameReader = this.kinectSensor.DepthFrameSource.OpenReader();

            // wire handler for frame arrival
            this.depthFrameReader.FrameArrived += this.Reader_FrameArrived;

            // get FrameDescription from DepthFrameSource
            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // allocate space to put the pixels being received and converted
            this.depthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];

            this.monochromePixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];

            this.colorPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height * sizeof(int)];

            // create the bitmap to display
            this.depthBitmapRaw = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray8, null);

            // better bitmap
            this.openCVBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray8, null);

            // setup the depth OpenCV image
            this.depthImage = new Image<Gray, byte>(512, 424, new Gray(20));
            // And draw some text on it.
            MCvFont f = new MCvFont(FONT.CV_FONT_HERSHEY_COMPLEX, 1.0, 1.0); //Create the font
            depthImage.Draw("Hello, world", ref f, new System.Drawing.Point(10, 80), new Gray(255)); //Draw "Hello, world." on the image using the specific font


            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // set up ZMQ stuff
            ZMQcontext = new ZContext();
            ZMQSocket = new ZSocket(ZMQcontext, ZSocketType.PUSH);
            // Connect
            ZMQSocket.Connect("tcp://127.0.0.1:1212");

            // set up PID stuff
            PIDTimer = new Timer();
            PIDTimer.Interval = PIDTickMS; //run at 20Hz;
            PIDTimer.Elapsed += DoPID;
            PIDTimer.Start();

            // initialize the components (controls) of the window
            this.InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.depthBitmapRaw;
            }
        }

        /// <summary>
        /// Convert an IImage to a WPF BitmapSource. The result can be used in the Set Property of Image.Source
        /// </summary>
        /// <param name="image">The Emgu CV Image</param>
        /// <returns>The equivalent BitmapSource</returns>
        public ImageSource OpenCVBitmapSource
        {
            get {
                return this.openCVBitmap;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            sendThrust(0);
            if (this.depthFrameReader != null)
            {
                // DepthFrameReader is IDisposable
                this.depthFrameReader.Dispose();
                this.depthFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        private void sendThrust(double thrust)
        {
            string req = @"{""ctrl"": {
                    ""version"": 1,
                    ""roll"": 0.0,
                    ""pitch"": 0.0,
                    ""yaw"": 0.0,
                    ""thrust"": "+thrust+"}}" + "\n";
            Console.Write("Sending {0}...", req);

            // Send
            ZMQSocket.Send(new ZFrame(req));
        }

        /// <summary>
        /// Handles the user clicking on the screenshot button
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void ScreenshotButton_Click(object sender, RoutedEventArgs e)
        {
            if (this.depthBitmapRaw != null)
            {
                // create a png bitmap encoder which knows how to save a .png file
                BitmapEncoder encoder = new PngBitmapEncoder();

                // create frame from the writable bitmap and add to encoder
                encoder.Frames.Add(BitmapFrame.Create(this.depthBitmapRaw));

                string time = System.DateTime.UtcNow.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

                string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);

                string path = Path.Combine(myPhotos, "KinectScreenshot-Depth-" + time + ".png");

                // write the new file to disk
                try
                {
                    // FileStream is IDisposable
                    using (FileStream fs = new FileStream(path, FileMode.Create))
                    {
                        encoder.Save(fs);
                    }

                    this.StatusText = string.Format(CultureInfo.CurrentCulture, Properties.Resources.SavedScreenshotStatusTextFormat, path);
                }
                catch (IOException)
                {
                    this.StatusText = string.Format(CultureInfo.CurrentCulture, Properties.Resources.FailedScreenshotStatusTextFormat, path);
                }
            }
        }

        /// <summary>
        /// Handles the depth frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, DepthFrameArrivedEventArgs e)
        {
            bool depthFrameProcessed = false;
           
            using (DepthFrame depthFrame = e.FrameReference.AcquireFrame())
            {
                if (depthFrame != null)
                {
                    // the fastest way to process the body index data is to directly access 
                    // the underlying buffer
                    using (Microsoft.Kinect.KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                    {
                        // verify data and write the color data to the display bitmap
                        if (((this.depthFrameDescription.Width * this.depthFrameDescription.Height) == (depthBuffer.Size / this.depthFrameDescription.BytesPerPixel)) &&
                            (this.depthFrameDescription.Width == this.depthBitmapRaw.PixelWidth) && (this.depthFrameDescription.Height == this.depthBitmapRaw.PixelHeight))
                        {
                            // Note: In order to see the full range of depth (including the less reliable far field depth)
                            // we are setting maxDepth to the extreme potential depth threshold
                            ushort maxDepth = (ushort)sliderMax.Value;

                            ushort minDepth = (ushort)sliderMin.Value;

                            double minBlob = (ushort)sliderMinSize.Value;

                            double maxBlob = (ushort)sliderMaxSize.Value;

                            // If you wish to filter by reliable depth distance, uncomment the following line:
                            //// maxDepth = depthFrame.DepthMaxReliableDistance
                            
                            this.ProcessDepthFrameData(depthBuffer.UnderlyingBuffer, depthBuffer.Size, minDepth, maxDepth, minBlob, maxBlob);
                           
                            depthFrameProcessed = true;
                        }
                    }
                }
            }

            if (depthFrameProcessed)
            {
                this.RenderDepthPixels();
            }
        }

        /// <summary>
        /// Directly accesses the underlying image buffer of the DepthFrame to 
        /// create a displayable bitmap.
        /// This function requires the /unsafe compiler option as we make use of direct
        /// access to the native memory pointed to by the depthFrameData pointer.
        /// </summary>
        /// <param name="depthFrameData">Pointer to the DepthFrame image data</param>
        /// <param name="depthFrameDataSize">Size of the DepthFrame image data</param>
        /// <param name="minDepth">The minimum reliable depth value for the frame</param>
        /// <param name="maxDepth">The maximum reliable depth value for the frame</param>
        private unsafe void ProcessDepthFrameData(IntPtr depthFrameData, uint depthFrameDataSize, ushort minDepth, ushort maxDepth, double minBlob, double maxBlob)
        {
            // depth frame data is a 16 bit value
            ushort* frameData = (ushort*)depthFrameData;

            // convert depth to a visual representation
            for (int i = 0; i < (int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel); ++i)
            {
                // Get the depth for this pixel
                ushort depth = frameData[i];

                // Values inside the depth range are mapped to 255 (white).
                // Values outside the reliable depth range are mapped to 0 (black).

                // You are having problems here because you should not be using the slider values in unsafe code?
                this.depthPixels[i] = (byte)(depth / MapDepthToByte);
                this.monochromePixels[i] = (byte)(depth >= (int)minDepth && depth <= (int)maxDepth ? 255 : 0);
            }

            this.depthImage.Bytes = monochromePixels;

            // And do contour stuff to it.

            // note: object creation here is probably a bad idea
            StringBuilder blobInfo = new StringBuilder();
            blobCount = 0;

            using (MemStorage stor = new MemStorage())
            {

                Contour<System.Drawing.Point> contours = depthImage.FindContours(
                Emgu.CV.CvEnum.CHAIN_APPROX_METHOD.CV_CHAIN_APPROX_SIMPLE,
                Emgu.CV.CvEnum.RETR_TYPE.CV_RETR_EXTERNAL,
                stor);

                for (int i = 0; contours != null; contours = contours.HNext)
                {
                    i++;

                    if ((contours.Area > minBlob) && (contours.Area < maxBlob))
                    {
                        MCvBox2D box = contours.GetMinAreaRect();
                        MCvMoments p = contours.GetMoments();
                        int x = (int)(p.m10 / p.m00);
                        int y = (int)(p.m01 / p.m00);
                        System.Drawing.PointF center = new System.Drawing.PointF(x, y);
                        CircleF circle = new CircleF(center, 2);
                        depthImage.Draw(box, new Gray(255), 2);
                        depthImage.Draw(circle, new Gray(127), 0);
                        blobInfo.Append("Blob ").Append(blobCount).Append(": \n");
                        blobInfo.Append("Area: ").Append((int)contours.Area).Append("\n");
                        blobInfo.Append("Center: ").Append(x).Append(", ").Append(y).Append("\n");
                        blobInfo.Append("\n");

                        if (blobCount == 0)
                        {
                            blob0_y = y;
                        }
                        blobCount++;
                    }
                }
            }

            
            txtBlobCount.Text = blobInfo.ToString();

        }


        //everything is a double because lazy
        double setpoint_y = 200;
        double error = 0;
        double lastError = 0;
        double integral = 0;
        double derivative;
        double PV;

        private void DoPID(object sender, System.Timers.ElapsedEventArgs eventArgs)
        {

            error = setpoint_y - blob0_y; // positive means that the craft is too high. negative means too low.

            integral += error /(double) PIDTickMS; // 

            if (integral > 1000)
            {
                integral = 1000;
            }

            if (integral < -1000)
            {
                integral = -1000;
            }

            derivative = error - lastError / (double) PIDTickMS; // if error > lasterror then this will be positive, indicating that the craft is rising

            lastError = error;


            //thrust is a scalefactor times the process variable. 
            thrust = 0.6 - 0.0005 * error - 0.0003 * integral;// +0.001 * derivative;
            sendThrust(thrust);

            this.StatusText = String.Format("Y: {0}, P: {1}, I: {2}, D: {3}, PV: {4}, Thrust: {5}", blob0_y, error, integral, derivative, PV, thrust);
            
        }
        /// <summary>
        /// Renders color pixels into the writeableBitmap.
        /// </summary>
        private void RenderDepthPixels()
        {
            this.depthBitmapRaw.WritePixels(
                new Int32Rect(0, 0, this.depthBitmapRaw.PixelWidth, this.depthBitmapRaw.PixelHeight),
                this.depthPixels,
                this.depthBitmapRaw.PixelWidth,
                0);

            this.openCVBitmap.WritePixels(
                new Int32Rect(0, 0, this.openCVBitmap.PixelWidth, this.openCVBitmap.PixelHeight),
                this.depthImage.Bytes,
                this.openCVBitmap.PixelWidth,
                0);
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            //this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
            //                                                : Properties.Resources.SensorNotAvailableStatusText;
        }
    }
}
