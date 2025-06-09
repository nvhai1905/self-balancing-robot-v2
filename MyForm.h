#pragma once
#include "FormButtonControl.h"
#include "FormGestureControl.h"


namespace bbb {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Summary for MyForm
	/// </summary>
	public ref class MyForm : public System::Windows::Forms::Form
	{
	public:
		MyForm(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~MyForm()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::Label^ label1;
	protected:
	private: System::Windows::Forms::Label^ label2;
	private: System::Windows::Forms::Label^ label3;
	private: System::Windows::Forms::PictureBox^ pictureBox1;
	private: System::Windows::Forms::Button^ btnButtonControl;
	private: System::Windows::Forms::Button^ btnGestureControl;
	private: System::Windows::Forms::PictureBox^ pictureBox2;

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>
		System::ComponentModel::Container ^components;

		System::Threading::Thread^ cameraThread;
		bool cameraRunning = false;
		TcpClient^ cameraClient;
		NetworkStream^ cameraStream;

#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			System::ComponentModel::ComponentResourceManager^ resources = (gcnew System::ComponentModel::ComponentResourceManager(MyForm::typeid));
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->pictureBox1 = (gcnew System::Windows::Forms::PictureBox());
			this->btnButtonControl = (gcnew System::Windows::Forms::Button());
			this->btnGestureControl = (gcnew System::Windows::Forms::Button());
			this->pictureBox2 = (gcnew System::Windows::Forms::PictureBox());
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox2))->BeginInit();
			this->SuspendLayout();
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->label1->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 16.2F, static_cast<System::Drawing::FontStyle>((System::Drawing::FontStyle::Bold | System::Drawing::FontStyle::Italic)),
				System::Drawing::GraphicsUnit::Point, static_cast<System::Byte>(163)));
			this->label1->Location = System::Drawing::Point(215, 33);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(381, 32);
			this->label1->TabIndex = 0;
			this->label1->Text = L"SELF BALANCING ROBOT";
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Italic, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(163)));
			this->label2->Location = System::Drawing::Point(64, 171);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(270, 25);
			this->label2->TabIndex = 1;
			this->label2->Text = L"Choose a control method........\r\n";
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(163)));
			this->label3->Location = System::Drawing::Point(297, 80);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(184, 72);
			this->label3->TabIndex = 2;
			this->label3->Text = L"Nguyen Van Hai 20226315\r\nNguyen Van Hai 20226314\r\nPham Van Doan 20226283\r\n\r\n";
			this->label3->Click += gcnew System::EventHandler(this, &MyForm::label3_Click);
			// 
			// pictureBox1
			// 
			this->pictureBox1->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"pictureBox1.Image")));
			this->pictureBox1->Location = System::Drawing::Point(659, 427);
			this->pictureBox1->Name = L"pictureBox1";
			this->pictureBox1->Size = System::Drawing::Size(147, 152);
			this->pictureBox1->SizeMode = System::Windows::Forms::PictureBoxSizeMode::Zoom;
			this->pictureBox1->TabIndex = 3;
			this->pictureBox1->TabStop = false;
			// 
			// btnButtonControl
			// 
			this->btnButtonControl->BackColor = System::Drawing::Color::IndianRed;
			this->btnButtonControl->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 13.8F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(163)));
			this->btnButtonControl->ForeColor = System::Drawing::SystemColors::ControlText;
			this->btnButtonControl->Location = System::Drawing::Point(246, 239);
			this->btnButtonControl->Name = L"btnButtonControl";
			this->btnButtonControl->Size = System::Drawing::Size(293, 73);
			this->btnButtonControl->TabIndex = 4;
			this->btnButtonControl->Text = L"Button Control";
			this->btnButtonControl->UseVisualStyleBackColor = false;
			this->btnButtonControl->Click += gcnew System::EventHandler(this, &MyForm::btnButtonControl_Click);
			// 
			// btnGestureControl
			// 
			this->btnGestureControl->BackColor = System::Drawing::Color::Gold;
			this->btnGestureControl->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 13.8F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(163)));
			this->btnGestureControl->Location = System::Drawing::Point(246, 386);
			this->btnGestureControl->Name = L"btnGestureControl";
			this->btnGestureControl->Size = System::Drawing::Size(293, 67);
			this->btnGestureControl->TabIndex = 5;
			this->btnGestureControl->Text = L"Gesture Control";
			this->btnGestureControl->UseVisualStyleBackColor = false;
			this->btnGestureControl->Click += gcnew System::EventHandler(this, &MyForm::btnGestureControl_Click);
			// 
			// pictureBox2
			// 
			this->pictureBox2->Location = System::Drawing::Point(659, 33);
			this->pictureBox2->Name = L"pictureBox2";
			this->pictureBox2->Size = System::Drawing::Size(147, 152);
			this->pictureBox2->SizeMode = System::Windows::Forms::PictureBoxSizeMode::Zoom;
			this->pictureBox2->TabIndex = 6;
			this->pictureBox2->TabStop = false;
			// 
			// MyForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(8, 16);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->BackColor = System::Drawing::Color::LightGray;
			this->ClientSize = System::Drawing::Size(804, 576);
			this->Controls->Add(this->pictureBox2);
			this->Controls->Add(this->btnGestureControl);
			this->Controls->Add(this->btnButtonControl);
			this->Controls->Add(this->pictureBox1);
			this->Controls->Add(this->label3);
			this->Controls->Add(this->label2);
			this->Controls->Add(this->label1);
			this->Name = L"MyForm";
			this->Text = L"MyForm";
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox2))->EndInit();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
	private: System::Void label3_Click(System::Object^ sender, System::EventArgs^ e) {
	}
	private: System::Void btnButtonControl_Click(System::Object^ sender, System::EventArgs^ e) {
		FormButtonControl^ formBtn = gcnew FormButtonControl();
		formBtn->Show();
	}

    private: System::Void btnGestureControl_Click(System::Object^ sender, System::EventArgs^ e) {
		FormGestureControl^ formGesture = gcnew FormGestureControl();
		formGesture->Show();
}

void StartCameraStream() {
    cameraRunning = true;
    cameraThread = gcnew System::Threading::Thread(gcnew System::Threading::ThreadStart(this, &MyForm::CameraStreamLoop));
    cameraThread->IsBackground = true;
    cameraThread->Start();
}

void CameraStreamLoop() {
    try {
        cameraClient = gcnew TcpClient();
        cameraClient->Connect("127.0.0.1", 9999);
        cameraStream = cameraClient->GetStream();

        while (cameraRunning) {
            // Read 4 bytes for length
            array<Byte>^ lenBytes = gcnew array<Byte>(4);
            int read = cameraStream->Read(lenBytes, 0, 4);
            if (read < 4) break;
            int frameLen = (lenBytes[0] << 24) | (lenBytes[1] << 16) | (lenBytes[2] << 8) | lenBytes[3];

            // Read the JPEG frame
            array<Byte>^ frameBytes = gcnew array<Byte>(frameLen);
            int totalRead = 0;
            while (totalRead < frameLen) {
                int n = cameraStream->Read(frameBytes, totalRead, frameLen - totalRead);
                if (n <= 0) break;
                totalRead += n;
            }
            if (totalRead < frameLen) break;

            // Convert to image and display
            System::IO::MemoryStream^ ms = gcnew System::IO::MemoryStream(frameBytes);
            System::Drawing::Image^ img = System::Drawing::Image::FromStream(ms);
            this->Invoke(gcnew System::Action<System::Drawing::Image^>(this, &MyForm::UpdateCameraImage), img);
        }
    }
    catch (Exception^ ex) {
        // Handle errors (optional)
    }
}

// Update the PictureBox with the new image
void UpdateCameraImage(System::Drawing::Image^ img) {
    if (pictureBox2->Image != nullptr) delete pictureBox2->Image;
    pictureBox2->Image = img;
}

// Stop the camera thread when closing the form
protected:
    virtual void OnFormClosing(FormClosingEventArgs^ e) override {
        cameraRunning = false;
        if (cameraThread != nullptr && cameraThread->IsAlive) cameraThread->Join(500);
        if (cameraStream != nullptr) cameraStream->Close();
        if (cameraClient != nullptr) cameraClient->Close();
        __super::OnFormClosing(e);
    }
};
}
