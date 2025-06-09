#pragma once

namespace bbb {
	using namespace System::Drawing;
	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace System::IO;
	using namespace System::Diagnostics;
	using namespace System::Threading;
	using namespace System::Net::Sockets;
	using namespace System::Net;
	using namespace System::IO::Ports;
	using namespace std;

	public ref class FormGestureControl : public System::Windows::Forms::Form
	{
	public:
		FormGestureControl(void)
		{
			InitializeComponent();
			serialPort = nullptr;
			receivedData = "";
			LoadCOMPorts();
		}

	protected:
		~FormGestureControl()
		{
			if (components)
			{
				delete components;
			}
		}

	private: System::Windows::Forms::PictureBox^ pictureBox1;
	private: System::Windows::Forms::Button^ button1;
	private: System::Windows::Forms::Timer^ timer1;
	private: System::Windows::Forms::PictureBox^ pictureBox2;
	private: System::Windows::Forms::Label^ labelStatus;

	private: System::Windows::Forms::Label^ label1;
	private: System::Windows::Forms::Label^ label2;
	private: System::ComponentModel::IContainer^ components;

	private: TcpClient^ videoClient;
	private: System::Windows::Forms::ComboBox^ comboBoxCOM;
	private: System::Windows::Forms::Button^ btnConnect;

	private: NetworkStream^ videoStream;

	private:
		SerialPort^ serialPort;
		String^ receivedData;

	private:
		bool gestureCommandsEnabled = false;

	private:
		void RunPythonScript() {
			Process^ process = gcnew Process();
			process->StartInfo->FileName = "python";
			process->StartInfo->Arguments = "\"D:\\KTLT_20242\\bbb.py\"";  // Thêm dấu ngoặc kép nếu có dấu cách
			process->StartInfo->UseShellExecute = false;
			process->StartInfo->CreateNoWindow = true;
			process->Start();
		}

		void UpdateImageLoop() {
			try {
				while (true) {
					Bitmap^ bmp = SafeLoadBitmap("h");
					if (bmp != nullptr) {
						this->BeginInvoke(gcnew Action<Bitmap^>(this, &FormGestureControl::UpdateImage), bmp);
					}
					// Optionally add a small sleep to avoid busy loop
					Threading::Thread::Sleep(1);
				}
			}
			catch (Exception^ ex) {
				// LogError("Error updating image from TCP", ex);
			}
		}

		//void LogError(String^ message, Exception^ ex) {
		//	try {
		//		String^ logMessage = DateTime::Now.ToString("yyyy-MM-dd HH:mm:ss") + " - " + message + "\r\n" +
		//			ex->ToString() + "\r\n";
		//		File::AppendAllText("error.log", logMessage);
		//	}
		//	catch (...) {
		//		// Suppress all exceptions from logging to avoid recursive failures
		//	}
		//}
	//private: System::Void UpdateImage(Bitmap^ image) {
	//	if (pictureBox1 != nullptr) {
	//		pictureBox1->Image = image;
	//	}
	//}

		Bitmap^ SafeLoadBitmap(String^ path) {
			if (videoStream == nullptr) return nullptr;

			try {
				// Read 4 bytes for the length (big-endian)
				array<Byte>^ lenBytes = gcnew array<Byte>(4);
				int read = videoStream->Read(lenBytes, 0, 4);
				if (read < 4) return nullptr;
				int len = (lenBytes[0] << 24) | (lenBytes[1] << 16) | (lenBytes[2] << 8) | lenBytes[3];

				// Read the image data
				array<Byte>^ imgBytes = gcnew array<Byte>(len);
				int offset = 0;
				while (offset < len) {
					int n = videoStream->Read(imgBytes, offset, len - offset);
					if (n == 0) return nullptr;
					offset += n;
				}

				MemoryStream^ ms = gcnew MemoryStream(imgBytes);
				Bitmap^ bmp = gcnew Bitmap(ms);
				ms->Close();
				return bmp;
			}
			catch (Exception^) {
				// Optionally log or handle errors
				return nullptr;
			}
		}

		void ConnectVideoStream() {
			try {
				videoClient = gcnew TcpClient("127.0.0.1", 9999);
				videoStream = videoClient->GetStream();
			}
			catch (Exception^ ex) {
				//LogError("Failed to connect to video stream", ex);*/
				videoClient = nullptr;
				videoStream = nullptr;
			}
		}

	private: System::Void button1_Click(System::Object^ sender, System::EventArgs^ e) {
		RunPythonScript();
		ConnectVideoStream();
		if (videoClient == nullptr || videoStream == nullptr) {
			labelStatus->Text = "Failed to connect to video stream";
			return;
		}

		Thread^ t = gcnew Thread(gcnew ThreadStart(this, &FormGestureControl::UpdateImageLoop));
		t->IsBackground = true;
		t->Start();
	}


	private: System::Void timer1_Tick(System::Object^ sender, System::EventArgs^ e) {
		try {
			// Cập nhật trạng thái từ gesture.txt
			if (File::Exists("gesture.txt")) {
				String^ content = File::ReadAllText("gesture.txt")->Trim();
				int fingerCount;
				if (!Int32::TryParse(content, fingerCount)) {
					labelStatus->Text = "Invalid data in gesture.txt";
					return;
				}

				switch (fingerCount) {
				case 1:
					labelStatus->Text = "Moving forward";
					SendCommand("F", "Moving forward");
					break;
				case 2:
					labelStatus->Text = "Moving backward";
					SendCommand("B", "Moving backward");
					break;
				case 3:
					labelStatus->Text = "Turning left";
					SendCommand("L", "Turning left");
					break;
				case 4:
					labelStatus->Text = "Turning right";
					SendCommand("R", "Turning right");
					break;
				default:
					labelStatus->Text = "Idle";
					break;
				}
			}
			else {
				labelStatus->Text = "gesture.txt not found";
			}

			// Cập nhật video từ frame.jpg

		}
		catch (IOException^ ex) {
			labelStatus->Text = "I/O Error: " + ex->Message;
		}
		catch (Exception^ ex) {
			labelStatus->Text = "Error: " + ex->Message;
		}
	}

	private: System::Void UpdateImage(Bitmap^ image) {
		if (pictureBox1 != nullptr) {
			pictureBox1->Image = image;
			pictureBox1->SizeMode = PictureBoxSizeMode::Zoom; // Giữ tỷ lệ hình ảnh
		}
	}

	private: System::Void FormGestureControl_Load(System::Object^ sender, System::EventArgs^ e) {
		timer1 = gcnew System::Windows::Forms::Timer();
		timer1->Interval = 33; // ~30 FPS
		timer1->Tick += gcnew System::EventHandler(this, &FormGestureControl::timer1_Tick);
		timer1->Start();
	}
	



#pragma region Windows Form Designer generated code
		   void InitializeComponent(void)
		   {
			   this->components = (gcnew System::ComponentModel::Container());
			   System::ComponentModel::ComponentResourceManager^ resources = (gcnew System::ComponentModel::ComponentResourceManager(FormGestureControl::typeid));
			   this->pictureBox1 = (gcnew System::Windows::Forms::PictureBox());
			   this->button1 = (gcnew System::Windows::Forms::Button());
			   this->timer1 = (gcnew System::Windows::Forms::Timer(this->components));
			   this->pictureBox2 = (gcnew System::Windows::Forms::PictureBox());
			   this->labelStatus = (gcnew System::Windows::Forms::Label());
			   this->label1 = (gcnew System::Windows::Forms::Label());
			   this->label2 = (gcnew System::Windows::Forms::Label());
			   this->comboBoxCOM = (gcnew System::Windows::Forms::ComboBox());
			   this->btnConnect = (gcnew System::Windows::Forms::Button());
			   (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->BeginInit();
			   (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox2))->BeginInit();
			   this->SuspendLayout();
			   // 
			   // pictureBox1
			   // 
			   this->pictureBox1->Location = System::Drawing::Point(2, 84);
			   this->pictureBox1->Name = L"pictureBox1";
			   this->pictureBox1->Size = System::Drawing::Size(602, 604);
			   this->pictureBox1->SizeMode = System::Windows::Forms::PictureBoxSizeMode::StretchImage;
			   this->pictureBox1->TabIndex = 0;
			   this->pictureBox1->TabStop = false;
			   // 
			   // button1
			   // 
			   this->button1->BackColor = System::Drawing::Color::Red;
			   this->button1->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				   static_cast<System::Byte>(163)));
			   this->button1->Location = System::Drawing::Point(67, 12);
			   this->button1->Name = L"button1";
			   this->button1->Size = System::Drawing::Size(174, 66);
			   this->button1->TabIndex = 1;
			   this->button1->Text = L"START";
			   this->button1->UseVisualStyleBackColor = false;
			   this->button1->Click += gcnew System::EventHandler(this, &FormGestureControl::button1_Click);
			   // 
			   // timer1
			   // 
			   this->timer1->Enabled = true;
			   this->timer1->Interval = 33;
			   this->timer1->Tick += gcnew System::EventHandler(this, &FormGestureControl::timer1_Tick);
			   // 
			   // pictureBox2
			   // 
			   this->pictureBox2->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"pictureBox2.Image")));
			   this->pictureBox2->Location = System::Drawing::Point(962, 590);
			   this->pictureBox2->Name = L"pictureBox2";
			   this->pictureBox2->Size = System::Drawing::Size(102, 123);
			   this->pictureBox2->SizeMode = System::Windows::Forms::PictureBoxSizeMode::Zoom;
			   this->pictureBox2->TabIndex = 2;
			   this->pictureBox2->TabStop = false;
			   // 
			   // labelStatus
			   // 
			   this->labelStatus->AutoSize = true;
			   this->labelStatus->BackColor = System::Drawing::SystemColors::ActiveCaption;
			   this->labelStatus->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 13.8F, static_cast<System::Drawing::FontStyle>((System::Drawing::FontStyle::Bold | System::Drawing::FontStyle::Italic)),
				   System::Drawing::GraphicsUnit::Point, static_cast<System::Byte>(163)));
			   this->labelStatus->Location = System::Drawing::Point(724, 310);
			   this->labelStatus->Name = L"labelStatus";
			   this->labelStatus->Size = System::Drawing::Size(115, 29);
			   this->labelStatus->TabIndex = 4;
			   this->labelStatus->Text = L"STATUS";
			   // 
			   // label1
			   // 
			   this->label1->AutoSize = true;
			   this->label1->BackColor = System::Drawing::SystemColors::ButtonShadow;
			   this->label1->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				   static_cast<System::Byte>(163)));
			   this->label1->Location = System::Drawing::Point(746, 58);
			   this->label1->Name = L"label1";
			   this->label1->Size = System::Drawing::Size(93, 50);
			   this->label1->TabIndex = 5;
			   this->label1->Text = L"Angle: 0\r\n\r\n";
			   // 
			   // label2
			   // 
			   this->label2->AutoSize = true;
			   this->label2->BackColor = System::Drawing::Color::IndianRed;
			   this->label2->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 13.8F, static_cast<System::Drawing::FontStyle>((System::Drawing::FontStyle::Bold | System::Drawing::FontStyle::Italic)),
				   System::Drawing::GraphicsUnit::Point, static_cast<System::Byte>(163)));
			   this->label2->Location = System::Drawing::Point(417, 12);
			   this->label2->Name = L"label2";
			   this->label2->Size = System::Drawing::Size(269, 29);
			   this->label2->TabIndex = 6;
			   this->label2->Text = L"GESTURE CONTROL";
			   // 
			   // comboBoxCOM
			   // 
			   this->comboBoxCOM->FormattingEnabled = true;
			   this->comboBoxCOM->Location = System::Drawing::Point(920, 157);
			   this->comboBoxCOM->Name = L"comboBoxCOM";
			   this->comboBoxCOM->Size = System::Drawing::Size(121, 24);
			   this->comboBoxCOM->TabIndex = 7;


			   // 
			   // btnConnect
			   // 
			   this->btnConnect->BackColor = System::Drawing::Color::Brown;
			   this->btnConnect->Location = System::Drawing::Point(920, 128);
			   this->btnConnect->Name = L"btnConnect";
			   this->btnConnect->Size = System::Drawing::Size(75, 23);
			   this->btnConnect->TabIndex = 8;
			   this->btnConnect->Text = L"Connect";
			   this->btnConnect->UseVisualStyleBackColor = false;
			   this->btnConnect->Click += gcnew System::EventHandler(this, &FormGestureControl::btnConnect_Click);

			   // 
			   // FormGestureControl
			   // 
			   this->AutoScaleDimensions = System::Drawing::SizeF(8, 16);
			   this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			   this->BackColor = System::Drawing::SystemColors::ButtonFace;
			   this->ClientSize = System::Drawing::Size(1062, 712);
			   this->Controls->Add(this->btnConnect);
			   this->Controls->Add(this->comboBoxCOM);
			   this->Controls->Add(this->label2);
			   this->Controls->Add(this->label1);
			   this->Controls->Add(this->labelStatus);
			   this->Controls->Add(this->pictureBox2);
			   this->Controls->Add(this->button1);
			   this->Controls->Add(this->pictureBox1);
			   this->Name = L"FormGestureControl";
			   this->Text = L"FormGestureControl";
			   (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->EndInit();
			   (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox2))->EndInit();
			   this->ResumeLayout(false);
			   this->PerformLayout();


		   }
#pragma endregion
	private:
		void LoadCOMPorts()
		{
			array<String^>^ ports = SerialPort::GetPortNames();
			comboBoxCOM->Items->Clear();
			for each(String ^ port in ports)
			{
				comboBoxCOM->Items->Add(port);
				Console::WriteLine("Found port: " + port);
			}
			if (comboBoxCOM->Items->Count > 0)
				comboBoxCOM->SelectedIndex = 0;
		}

		void SetGestureCommandsEnabled(bool enabled)
		{
			// If you have gesture-related buttons, enable/disable them here.
			// If gestures are sent automatically (e.g., in timer1_Tick), use a flag:
			gestureCommandsEnabled = enabled;
		}

		void SendCommand(String^ command, String^ displayText)
		{
			if (!gestureCommandsEnabled)
				return; // Ignore command if not enabled

			if (serialPort != nullptr && serialPort->IsOpen)
			{
				try
				{
					serialPort->WriteLine(command);
					labelStatus->Text = displayText + "...";
				}
				catch (Exception^ ex)
				{
					labelStatus->Text = "Serial error: " + ex->Message;
					SetGestureCommandsEnabled(false);
				}
			}
			else
			{
				SetGestureCommandsEnabled(false);
				// Optionally: MessageBox::Show("Please connect to a COM port first!", ...);
			}
		}


		// Hàm xử lý khi nhận dữ liệu từ Serial
		void SerialPort_DataReceived(Object^ sender, SerialDataReceivedEventArgs^ e)
		{
			try
			{
				String^ data = serialPort->ReadLine();
				receivedData += data + "\n";

				// Tìm và xử lý dữ liệu góc
				array<String^>^ lines = receivedData->Split(gcnew array<String^>{"\n"}, StringSplitOptions::RemoveEmptyEntries);
				for each(String ^ line in lines)
				{
					if (line->StartsWith("ANGLE:"))
					{
						String^ angleStr = line->Substring(6); // Bỏ phần "ANGLE:"
						double angle;
						if (Double::TryParse(angleStr, angle))
						{
							// Cập nhật label2 trên luồng giao diện
							label2->Invoke(gcnew Action<double>(this, &FormGestureControl::UpdateAngleLabel), angle);
						}
					}
				}
				receivedData = ""; // Xóa dữ liệu đã xử lý
			}
			catch (Exception^ ex)
			{
				MessageBox::Show("Error reading from Serial: " + ex->Message, "Serial Error", MessageBoxButtons::OK, MessageBoxIcon::Error);
			}
		}

		// Hàm cập nhật label2 với giá trị góc
		void UpdateAngleLabel(double angle)
		{
			label2->Text = "Angle: " + angle.ToString("F2"); // Hiển thị với 2 chữ số thập phân
		}

	private: System::Void btnConnect_Click(System::Object^ sender, System::EventArgs^ e) {
		try
		{
			if (serialPort != nullptr && serialPort->IsOpen)
			{
				serialPort->Close();
			}

			if (comboBoxCOM->SelectedItem == nullptr)
			{
				MessageBox::Show("Please select a COM port!", "Selection Error", MessageBoxButtons::OK, MessageBoxIcon::Warning);
				return;
			}

			String^ selectedPort = comboBoxCOM->SelectedItem->ToString();
			serialPort = gcnew SerialPort(selectedPort, 115200);
			serialPort->ReadTimeout = 500;
			serialPort->WriteTimeout = 500;
			serialPort->DataReceived += gcnew SerialDataReceivedEventHandler(this, &FormGestureControl::SerialPort_DataReceived);
			serialPort->Open();
			labelStatus->Text = "Connected to " + selectedPort;
			MessageBox::Show("Connected successfully to " + selectedPort, "Success", MessageBoxButtons::OK, MessageBoxIcon::Information);
		}
		catch (Exception^ ex)
		{
			MessageBox::Show("Connection error: " + ex->Message + "\nPort: " + (comboBoxCOM->SelectedItem != nullptr ? comboBoxCOM->SelectedItem->ToString() : "None"), "Error", MessageBoxButtons::OK, MessageBoxIcon::Error);
		}
	}
	};
};

