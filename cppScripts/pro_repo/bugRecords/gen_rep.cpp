#include "generation_report.h"
#include <QPainter>
#include <QFile>
#include <QtPrintSupport/QtPrintSupport>
#pragma execution_character_set("utf-8")


void GenRep::CreatePdfAccuRepeatSlot()
{
	setTipWindow("���Ӳ���׼ȷ�Ⱥ��ظ��Բ��Ա�����������...");
	QString str = file_path + "tele_acc.log";
	QFile file(str);

	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
		setTipWindow("���Ӳ���׼ȷ�Ⱥ��ظ��Բ��Ա�������ʧ��");
		return;
	}
	//�������ǰ·��
	QString currentDir = QDir::currentPath();
	//����pdf�����·��
	QString file_path = QFileDialog::getExistingDirectory(this, "��ѡ�񱣴�·��", "C:\\Users\\Administrator\\Desktop");
	if (file_path.isEmpty())
	{
		QMessageBox::warning(this, "��ʾ", "·������Ϊ��");
		setTipWindow("���Ӳ���׼ȷ�Ⱥ��ظ��Բ��Ա�������ʧ��");
		return;
		
	}

	//�������ִ��·�����õ�filePath��
	QDir tempDir;
	tempDir.setCurrent(file_path);

	//������
	QProgressBar* m_pProgressBar = new QProgressBar(this);
	m_pProgressBar->setOrientation(Qt::Horizontal);  // ˮƽ����
	m_pProgressBar->setMinimum(1);  // ��Сֵ
	m_pProgressBar->setMaximum(1);  // ���ֵ str.size()
	m_pProgressBar->setMaximumHeight(15);
	m_pProgressBar->setMinimumWidth(60);
	m_pProgressBar->setVisible(true);


	m_pProgressBar->setValue(0);
	QCoreApplication::processEvents();

	QDate date = QDate::currentDate();

	//QString serialNum = QString::number(Production.at(ui.toolComboBox->currentIndex()).second);
	//QString year = str.at(3).left(4);

	QString fileName = QString::fromStdString("���Ӳ���׼ȷ�Ⱥ��ظ��Բ���") + ".pdf";
	QFile pdfFile(fileName);
	//�ж��ļ��Ƿ����
	if (QFile::exists(fileName))
	{
		QMessageBox::StandardButton reply;
		reply = QMessageBox::question(this, "��ʾ", "�ļ��Ѿ�����,����ǽ�����ԭ�ļ�", QMessageBox::Yes | QMessageBox::No);
		if (reply == QMessageBox::Yes)
		{
			QFile::remove(fileName);
		}
		else
		{
			setTipWindow("���Ӳ���׼ȷ�Ⱥ��ظ��Բ��Ա�������ʧ��");
			return;
		}
	}
	if (1)
	{
		// ��Ҫд���pdf�ļ�
		pdfFile.open(QIODevice::WriteOnly);

		QPdfWriter* pPdfWriter = new QPdfWriter(&pdfFile);  // ����pdfд����
		pPdfWriter->setPageSize(QPagedPaintDevice::A4);     // ����ֽ��ΪA4
		pPdfWriter->setResolution(300);                     // ����ֽ�ŵķֱ���Ϊ300,���������Ϊ3508X2479

		int iMargin = 0;                   // ҳ�߾�
		pPdfWriter->setPageMargins(QMarginsF(iMargin, iMargin, iMargin, iMargin));

		QPainter* pPdfPainter = new QPainter(pPdfWriter);   // qt���ƹ���

		// ����,����
		QTextOption option(Qt::AlignHCenter | Qt::AlignVCenter);
		option.setWrapMode(QTextOption::WordWrap);

		//����
		//QFont font;
		//font.setFamily("simhei.ttf");
		QFont font[8] = { QFont("����", 26, 60), QFont("����", 26, 61), QFont("����", 26, QFont::Normal), QFont("times new roman", 26, QFont::Normal), QFont("����", 26, QFont::Normal), QFont("������κ", 26, 61), QFont("����", 26, QFont::Normal), QFont("times new roman", 26, QFont::Normal) };
		font[0].setPixelSize(86);
		font[1].setPixelSize(55);
		font[2].setPixelSize(40);
		font[3].setPixelSize(40);
		font[4].setPixelSize(50);
		font[5].setPixelSize(50);
		font[6].setPixelSize(40);
		font[7].setPixelSize(50);
		//Painter PDF
		qDebug() << pPdfPainter->viewport();
		int nPDFWidth = pPdfPainter->viewport().width();
		int nPDFHeight = pPdfPainter->viewport().height();

		//pPdfPainter->setPen(QPen(Qt::black, 5));
		//pPdfPainter->drawLine(0, 0, 2478, 0);
		//pPdfPainter->size
		//����ͼƬˮƽ�߾�Ϊ150
		//pPdfPainter->setOpacity(0.4);
		//pPdfPainter->drawPixmap(5000, 100, 220 * 1.3, 110 * 1.3, pixmap);
		//pPdfPainter->drawPixmap(200, 100, 220 * 1.3, 110 * 1.3, pixmap);
		//pPdfPainter->setOpacity(1.0);


		//ͼֽ��С
		int pdfWidth = 2478;
		int pdfHeight = 3508;
		pPdfPainter->setFont(font[2]);
		pPdfPainter->setPen(QPen(Qt::darkGray, 1));




		pPdfPainter->drawLine(150, 270, 2328, 270);  //2478-150

		pPdfPainter->setFont(font[5]);
		pPdfPainter->setPen(QPen(Qt::black, 2));
		pPdfPainter->drawText(QRect(0, 0, 2478 / 3, 250), Qt::AlignHCenter | Qt::AlignBottom,
			"�������������޹�˾");


		//���Ʊ�����Ϣ
		int chartW1 = 2478 - 400;
		int chartH1 = 950;
		int chartL1 = 200;
		int chartT1 = 550;
		int chartRowH1 = 85;
		int charti1 = 0;
		pPdfPainter->setFont(font[1]);
		pPdfPainter->setPen(QPen(Qt::black, 3));

		pPdfPainter->drawText(QRect(150, 300, 2328, 200), Qt::AlignHCenter | Qt::AlignVCenter,
			QString("%1����").arg("���Ӳ���׼ȷ�Ⱥ��ظ��Բ���"));

		pPdfPainter->setFont(font[4]);
		pPdfPainter->setPen(QPen(Qt::black, 5));
		pPdfPainter->drawRect(chartL1, chartT1, chartW1, chartH1);
		pPdfPainter->setPen(QPen(Qt::black));
		pPdfPainter->setFont(font[2]);
		chartL1 += 15;
		pPdfPainter->drawText(QRect(chartL1 + 90, chartRowH1 * -1 + chartT1 + 12, pdfWidth / 4, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "���� ��9527");
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 340, chartRowH1 * -1 + chartT1 + 12, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "��� ��         ");

		pPdfPainter->setFont(font[6]);
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("            %1").arg("�ڿ�������ϵͳ"));

		pPdfPainter->setFont(font[2]);
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 260, chartRowH1 * charti1 + chartT1 + 10, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "�� �� Ա ��         ");
		charti1++;
		pPdfPainter->drawLine(pdfWidth / 2 + 500, chartRowH1 * charti1 + chartT1 + 5, chartW1 + 100, chartRowH1 * charti1 + chartT1 + 5);

		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("��Ʒ�ͺ� ��%1").arg(system_proc));
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 260, chartRowH1 * charti1 + chartT1 + 10, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "�������� ��         ");
		charti1++;
		pPdfPainter->drawLine(pdfWidth / 2 + 500, chartRowH1 * charti1 + chartT1 + 5, chartW1 + 100, chartRowH1 * charti1 + chartT1 + 5);
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�� �� �� ��%1").arg(system_serial));
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 260, chartRowH1 * charti1 + chartT1 + 10, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "�� �� Ա ��         ");
		charti1++;
		pPdfPainter->drawLine(pdfWidth / 2 + 500, chartRowH1 * charti1 + chartT1 + 5, chartW1 + 100, chartRowH1 * charti1 + chartT1 + 5);
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�������� ��%1").arg(system_batch));
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 260, chartRowH1 * charti1 + chartT1 + 10, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "������� ��         ");
		charti1++;
		pPdfPainter->drawLine(pdfWidth / 2 + 500, chartRowH1 * charti1 + chartT1 + 5, chartW1 + 100, chartRowH1 * charti1 + chartT1 + 5);
		//pPdfPainter->drawLine(chartL1 + 200, chartRowH1*charti1 + chartT1 + 10, chartW1 + 100, chartRowH1*charti1 + chartT1 + 10);


		pPdfPainter->setFont(font[6]);
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("            %1").arg("����ǯ"));
		pPdfPainter->setFont(font[2]);
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("��Ʒ�ͺ� ��%1").arg(tool_proc));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�� �� �� ��%1").arg(tool_serial));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�������� ��%1").arg(tool_batch));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�豸���� ��%1").arg(QString::fromStdString("���Ӳ���׼ȷ�Ⱥ��ظ��Բ��Թ�װ")));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�豸��� ��%1").arg(QString::fromStdString(" ")));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�����׼ ��%1").arg(QString::fromStdString("������ SR-ENS-600 ����Ҫ��")));
		//pPdfPainter->drawLine(pdfWidth / 2 - 160, chartRowH1*charti1 + chartT1 + 10, chartW1 - 100, chartRowH1*charti1 + chartT1 + 10);
		pPdfPainter->setFont(font[2]);

		//��log
		qreal array[25][4] = { 0 };



		QTextStream in(&file);

		int line_number = 0;
		while (!in.atEnd()) {
			QString line = in.readLine();
			QStringList str_list = line.split(QRegExp("\\s{1,}"));
			for (int i = 0; i < str_list.length(); ++i) {
				array[line_number][i] = str_list[i].toFloat();
			}
			++line_number;
		}
		file.close();

		//��񡪡�����׼ȷ�Ȳ��Խ��
		int chartW2 = 2478 - 400;
		int chartH2 = 700;
		int chartL2 = 200;
		int chartT2 = 1600;
		int chartRowH2 = 70;
		int charti2 = 0;
		pPdfPainter->setPen(QPen(Qt::black, 5));
		pPdfPainter->drawRect(chartL2, chartT2, chartW2, chartH2);

		//pPdfPainter->setFont(font[1]);
		//pPdfPainter->drawText(QRect(chartL2, chartT2 - 159, chartW2, 150), Qt::AlignHCenter | Qt::AlignVCenter,
		//	"׼ȷ�Ⱥ��ظ��Ȳ���");

		pPdfPainter->setPen(QPen(Qt::black, 3));

		pPdfPainter->setFont(font[3]);

		pPdfPainter->drawText(QRect(chartL2, chartT2, chartW2, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			" ����׼ȷ�Ȳ��Խ������λ��mm��");

		charti2++;
		pPdfPainter->drawText(QRect(chartL2, chartRowH2 * charti2 + chartT2, 278, chartRowH2 * 2), Qt::AlignHCenter | Qt::AlignVCenter,
			" �Խ��� ");
		pPdfPainter->drawText(QRect(chartL2 + 278, chartRowH2 * charti2 + chartT2 + 5, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"����׼ȷ��");
		pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 1, chartRowH2 * charti2 + chartT2 + 5, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"X�������׼");
		pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 2, chartRowH2 * charti2 + chartT2 + 5, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"Y�������׼");
		pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 3, chartRowH2 * charti2 + chartT2 + 5, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"Z�������׼");

		charti2++;
		pPdfPainter->drawText(QRect(chartL2 + 278, chartRowH2 * charti2 + chartT2 - 5, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"|ADp|");
		pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 1, chartRowH2 * charti2 + chartT2 - 5, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"ȷ��|ADx|");
		pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 2, chartRowH2 * charti2 + chartT2 - 5, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"ȷ��|ADy|");
		pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 3, chartRowH2 * charti2 + chartT2 - 5, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"ȷ��|ADz|");

		charti2++;
		pPdfPainter->drawText(QRect(chartL2, chartRowH2 * charti2 + chartT2, 278, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			" AG ");
		pPdfPainter->drawText(QRect(chartL2 + 278, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[0][0], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 1, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[0][1], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 2, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[0][2], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 3, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[0][3], 'f', 2)));

		charti2++;
		pPdfPainter->drawText(QRect(chartL2, chartRowH2 * charti2 + chartT2, 278, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			" BH ");
		pPdfPainter->drawText(QRect(chartL2 + 278, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[1][0], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 1, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[1][1], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 2, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[1][2], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 3, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[1][3], 'f', 2)));

		charti2++;
		pPdfPainter->drawText(QRect(chartL2, chartRowH2 * charti2 + chartT2, 278, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			" CE ");
		pPdfPainter->drawText(QRect(chartL2 + 278, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[2][0], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 1, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[2][1], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 2, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[2][2], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 3, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[2][3], 'f', 2)));

		charti2++;
		pPdfPainter->drawText(QRect(chartL2, chartRowH2 * charti2 + chartT2, 278, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			" DF ");
		pPdfPainter->drawText(QRect(chartL2 + 278, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[3][0], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 1, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[3][1], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 2, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[3][2], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 3, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[3][3], 'f', 2)));

		//���ֵ
		QVector <double> data0{ array[0][0], array[1][0], array[2][0], array[3][0] };
		auto max0 = std::max_element(std::begin(data0), std::end(data0));
		double biggest0 = *max0;

		QVector <double> data1{ array[0][1], array[1][1], array[2][1], array[3][1] };
		auto max1 = std::max_element(std::begin(data1), std::end(data1));
		double biggest1 = *max1;

		QVector <double> data2{ array[0][2], array[1][2], array[2][2], array[3][2] };
		auto max2 = std::max_element(std::begin(data2), std::end(data2));
		double biggest2 = *max2;

		QVector <double> data3{ array[0][3], array[1][3], array[2][3], array[3][3] };
		auto max3 = std::max_element(std::begin(data3), std::end(data3));
		double biggest3 = *max3;

		charti2++;
		pPdfPainter->drawText(QRect(chartL2, chartRowH2 * charti2 + chartT2, 278, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			" ���ֵ ");
		pPdfPainter->drawText(QRect(chartL2 + 278, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(biggest0, 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 1, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(biggest1, 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 2, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(biggest2, 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 3, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(biggest3, 'f', 2)));

		charti2++;
		pPdfPainter->drawText(QRect(chartL2, chartRowH2 * charti2 + chartT2, 278, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			" Ҫ�� ");
		pPdfPainter->drawText(QRect(chartL2 + 278, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			" �� 2 ");
		pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 1, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			" �� 2 ");
		pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 2, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			" �� 2 ");
		pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 3, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			" �� 2 ");

		charti2++;
		pPdfPainter->drawText(QRect(chartL2, chartRowH2 * charti2 + chartT2, 278, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			" �Ƿ�ͨ�� ");
		if (biggest0 <= 2)
		{
			pPdfPainter->drawText(QRect(chartL2 + 278, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				QString(" %1 ").arg("ͨ��"));
		}

		else
		{
			pPdfPainter->drawText(QRect(chartL2 + 278, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				QString(" %1 ").arg("δͨ��"));
		}

		if (biggest1 <= 2)
		{
			pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 1, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				QString(" %1 ").arg("ͨ��"));
		}

		else
		{
			pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 1, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				QString(" %1 ").arg("δͨ��"));
		}

		if (biggest2 <= 2)
		{
			pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 2, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				QString(" %1 ").arg("ͨ��"));
		}

		else
		{
			pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 2, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				QString(" %1 ").arg("δͨ��"));
		}
		if (biggest3 <= 2)
		{
			pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 3, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				QString(" %1 ").arg("ͨ��"));
		}

		else
		{
			pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 3, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				QString(" %1 ").arg("δͨ��"));
		}


		//����
		pPdfPainter->drawLine(chartL2 + 278, chartT2 + chartRowH2, chartL2 + 278, chartT2 + chartH2);
		pPdfPainter->drawLine(chartL2 + 278 + 450 * 1, chartT2 + chartRowH2, chartL2 + 278 + 450 * 1, chartT2 + chartH2);
		pPdfPainter->drawLine(chartL2 + 278 + 450 * 2, chartT2 + chartRowH2, chartL2 + 278 + 450 * 2, chartT2 + chartH2);
		pPdfPainter->drawLine(chartL2 + 278 + 450 * 3, chartT2 + chartRowH2, chartL2 + 278 + 450 * 3, chartT2 + chartH2);
		//pPdfPainter->drawLine(chartL2 + 278 + 450 * 4, chartT2, chartL2 + 278 + 450 * 4, chartT2 + chartH2);
		//pPdfPainter->drawLine(chartL2 + 278 + 450 * 5, chartT2, chartL2 + 278 + 450 * 5, chartT2 + chartH2);

		//����
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2, chartL2 + chartW2, chartT2 + chartRowH2);
		//pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 2, chartL2 + chartW2, chartT2 + chartRowH2 * 2);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 3, chartL2 + chartW2, chartT2 + chartRowH2 * 3);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 4, chartL2 + chartW2, chartT2 + chartRowH2 * 4);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 5, chartL2 + chartW2, chartT2 + chartRowH2 * 5);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 6, chartL2 + chartW2, chartT2 + chartRowH2 * 6);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 7, chartL2 + chartW2, chartT2 + chartRowH2 * 7);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 8, chartL2 + chartW2, chartT2 + chartRowH2 * 8);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 9, chartL2 + chartW2, chartT2 + chartRowH2 * 9);




		//��񡪡������ظ��Բ��Խ��
		int chartW3 = 2478 - 400;
		int chartH3 = 700;
		int chartL3 = 200;
		int chartT3 = 2400;
		int chartRowH3 = 70;
		int charti3 = 0;
		pPdfPainter->setPen(QPen(Qt::black, 5));
		pPdfPainter->drawRect(chartL3, chartT3, chartW3, chartH3);

		//pPdfPainter->setFont(font[1]);
		//pPdfPainter->drawText(QRect(chartL3, chartT3 - 159, chartW3, 150), Qt::AlignHCenter | Qt::AlignVCenter,
		//	"׼ȷ�Ⱥ��ظ��Ȳ���");

		pPdfPainter->setPen(QPen(Qt::black, 3));

		pPdfPainter->setFont(font[3]);

		pPdfPainter->drawText(QRect(chartL3, chartT3, chartW3, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			" �����ظ��Բ��Խ������λ��mm��");

		charti3++;
		pPdfPainter->drawText(QRect(chartL3, chartRowH3 * charti3 + chartT3, 278, chartRowH3 * 2), Qt::AlignHCenter | Qt::AlignVCenter,
			" �Խ��� ");
		pPdfPainter->drawText(QRect(chartL3 + 278, chartRowH3 * charti3 + chartT3 + 5, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			"�����ظ���");
		pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 1, chartRowH3 * charti3 + chartT3 + 5, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			"X�������׼");
		pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 2, chartRowH3 * charti3 + chartT3 + 5, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			"Y�������׼");
		pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 3, chartRowH3 * charti3 + chartT3 + 5, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			"Z�������׼");

		charti3++;
		pPdfPainter->drawText(QRect(chartL3 + 278, chartRowH3 * charti3 + chartT3 - 5, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			"RD(��)");
		pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 1, chartRowH3 * charti3 + chartT3 - 5, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			"ȷ��RDx(��)");
		pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 2, chartRowH3 * charti3 + chartT3 - 5, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			"ȷ��RDy(��)");
		pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 3, chartRowH3 * charti3 + chartT3 - 5, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			"ȷ��RDz(��)");

		charti3++;
		pPdfPainter->drawText(QRect(chartL3, chartRowH3 * charti3 + chartT3, 278, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			" AG ");
		pPdfPainter->drawText(QRect(chartL3 + 278, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[4][0], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 1, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[4][1], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 2, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[4][2], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 3, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[4][3], 'f', 2)));

		charti3++;
		pPdfPainter->drawText(QRect(chartL3, chartRowH3 * charti3 + chartT3, 278, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			" BH ");
		pPdfPainter->drawText(QRect(chartL3 + 278, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[5][0], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 1, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[5][1], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 2, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[5][2], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 3, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[5][3], 'f', 2)));

		charti3++;
		pPdfPainter->drawText(QRect(chartL3, chartRowH3 * charti3 + chartT3, 278, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			" CE ");
		pPdfPainter->drawText(QRect(chartL3 + 278, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[6][0], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 1, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[6][1], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 2, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[6][2], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 3, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[6][3], 'f', 2)));

		charti3++;
		pPdfPainter->drawText(QRect(chartL3, chartRowH3 * charti3 + chartT3, 278, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			" DF ");
		pPdfPainter->drawText(QRect(chartL3 + 278, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[7][0], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 1, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[7][1], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 2, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[7][2], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 3, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[7][3], 'f', 2)));

		//���ֵ
		QVector <double> data4{ array[4][0], array[5][0], array[6][0], array[7][0] };
		auto max4 = std::max_element(std::begin(data4), std::end(data4));
		double biggest4 = *max4;

		QVector <double> data5{ array[4][1], array[5][1], array[6][1], array[7][1] };
		auto max5 = std::max_element(std::begin(data5), std::end(data5));
		double biggest5 = *max5;

		QVector <double> data6{ array[4][2], array[5][2], array[6][2], array[7][2] };
		auto max6 = std::max_element(std::begin(data6), std::end(data6));
		double biggest6 = *max6;

		QVector <double> data7{ array[4][3], array[5][3], array[6][3], array[7][3] };
		auto max7 = std::max_element(std::begin(data7), std::end(data7));
		double biggest7 = *max7;


		charti3++;
		pPdfPainter->drawText(QRect(chartL3, chartRowH3 * charti3 + chartT3, 278, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			" ���ֵ ");
		pPdfPainter->drawText(QRect(chartL3 + 278, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(biggest4, 'f', 2)));
		pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 1, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(biggest5, 'f', 2)));
		pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 2, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(biggest6, 'f', 2)));
		pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 3, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(biggest7, 'f', 2)));

		charti3++;
		pPdfPainter->drawText(QRect(chartL3, chartRowH3 * charti3 + chartT3, 278, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			" Ҫ�� ");
		pPdfPainter->drawText(QRect(chartL3 + 278, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			" �� 2 ");
		pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 1, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			" �� 2 ");
		pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 2, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			" �� 2 ");
		pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 3, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			" �� 2 ");

		charti3++;
		pPdfPainter->drawText(QRect(chartL3, chartRowH3 * charti3 + chartT3, 278, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			" �Ƿ�ͨ�� ");
		if (biggest4 <= 2)
		{
			pPdfPainter->drawText(QRect(chartL3 + 278, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				QString(" %1 ").arg("ͨ��"));
		}

		else
		{
			pPdfPainter->drawText(QRect(chartL3 + 278, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				QString(" %1 ").arg("δͨ��"));
		}

		if (biggest5 <= 2)
		{
			pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 1, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				QString(" %1 ").arg("ͨ��"));
		}

		else
		{
			pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 1, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				QString(" %1 ").arg("δͨ��"));
		}

		if (biggest6 <= 2)
		{
			pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 2, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				QString(" %1 ").arg("ͨ��"));
		}

		else
		{
			pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 2, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				QString(" %1 ").arg("δͨ��"));
		}
		if (biggest7 <= 2)
		{
			pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 3, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				QString(" %1 ").arg("ͨ��"));
		}

		else
		{
			pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 3, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				QString(" %1 ").arg("δͨ��"));
		}


		//����
		pPdfPainter->drawLine(chartL3 + 278, chartT3 + chartRowH3, chartL3 + 278, chartT3 + chartH3);
		pPdfPainter->drawLine(chartL3 + 278 + 450 * 1, chartT3 + chartRowH3, chartL3 + 278 + 450 * 1, chartT3 + chartH3);
		pPdfPainter->drawLine(chartL3 + 278 + 450 * 2, chartT3 + chartRowH3, chartL3 + 278 + 450 * 2, chartT3 + chartH3);
		pPdfPainter->drawLine(chartL3 + 278 + 450 * 3, chartT3 + chartRowH3, chartL3 + 278 + 450 * 3, chartT3 + chartH3);
		//pPdfPainter->drawLine(chartL3 + 278 + 450 * 4, chartT3, chartL3 + 278 + 450 * 4, chartT3 + chartH3);
		//pPdfPainter->drawLine(chartL3 + 278 + 450 * 5, chartT3, chartL3 + 278 + 450 * 5, chartT3 + chartH3);

		//����
		pPdfPainter->drawLine(chartL3, chartT3 + chartRowH3, chartL3 + chartW3, chartT3 + chartRowH3);
		//pPdfPainter->drawLine(chartL3, chartT3 + chartRowH3 * 2, chartL3 + chartW3, chartT3 + chartRowH3 * 2);
		pPdfPainter->drawLine(chartL3, chartT3 + chartRowH3 * 3, chartL3 + chartW3, chartT3 + chartRowH3 * 3);
		pPdfPainter->drawLine(chartL3, chartT3 + chartRowH3 * 4, chartL3 + chartW3, chartT3 + chartRowH3 * 4);
		pPdfPainter->drawLine(chartL3, chartT3 + chartRowH3 * 5, chartL3 + chartW3, chartT3 + chartRowH3 * 5);
		pPdfPainter->drawLine(chartL3, chartT3 + chartRowH3 * 6, chartL3 + chartW3, chartT3 + chartRowH3 * 6);
		pPdfPainter->drawLine(chartL3, chartT3 + chartRowH3 * 7, chartL3 + chartW3, chartT3 + chartRowH3 * 7);
		pPdfPainter->drawLine(chartL3, chartT3 + chartRowH3 * 8, chartL3 + chartW3, chartT3 + chartRowH3 * 8);
		pPdfPainter->drawLine(chartL3, chartT3 + chartRowH3 * 9, chartL3 + chartW3, chartT3 + chartRowH3 * 9);


		//ҳ��
		pPdfPainter->setFont(font[3]);
		pPdfPainter->setPen(QPen(Qt::darkGray, 1));
		pPdfPainter->drawLine(150, 3310, 2328, 3310);  //2478-150

		pPdfPainter->drawText(QRect(1100, 3200, 300, 300), Qt::AlignHCenter | Qt::AlignVCenter,
			"1/2");
		pPdfPainter->drawText(QRect(1820, 3200, 570, 300), Qt::AlignHCenter | Qt::AlignVCenter,
			"������Ϣ �ϸ���");
		pPdfPainter->drawText(QRect(50, 3200, 700, 300), Qt::AlignHCenter | Qt::AlignVCenter,
			QString("%1").arg("SR-ENS-600"));
		pPdfPainter->setFont(font[2]);



		//�ڶ�ҳ
		pPdfWriter->newPage();

		/*		pPdfPainter->setOpacity(0.4);
				pPdfPainter->drawPixmap(200, 100, 220 * 1.3, 110 * 1.3, pixmap);
				pPdfPainter->setOpacity(1.0);
				pPdfPainter->setFont(font[2])*/;
				pPdfPainter->setPen(QPen(Qt::darkGray, 1));
				//pPdfPainter->drawText(QRect(0, 0, 2478/2, 250), Qt::AlignHCenter | Qt::AlignBottom,
				//	"�������������޹�˾");
				//pPdfPainter->drawText(QRect(1500, 0, 2478 - 1500, 250), Qt::AlignHCenter | Qt::AlignBottom,
				//	QString("�ļ���� ��%1").arg(str.at(2)));
				pPdfPainter->drawLine(150, 270, 2328, 270);  //2478-150

				pPdfPainter->setFont(font[5]);
				pPdfPainter->setPen(QPen(Qt::black, 2));
				pPdfPainter->drawText(QRect(0, 0, 2478 / 3, 250), Qt::AlignHCenter | Qt::AlignBottom,
					"�������������޹�˾");

				pPdfPainter->setPen(QPen(Qt::black, 1));
				pPdfPainter->setFont(font[2]);


				//��񡪡���̬׼ȷ�Ȳ��Խ��
				int chartW4 = 2478 - 400;
				int chartH4 = 980;
				int chartL4 = 200;
				int chartT4 = 350;
				int chartRowH4 = 70;
				int charti4 = 0;
				pPdfPainter->setPen(QPen(Qt::black, 5));
				pPdfPainter->drawRect(chartL4, chartT4, chartW4, chartH4);

				//pPdfPainter->setFont(font[1]);
				//pPdfPainter->drawText(QRect(chartL4, chartT4 - 159, chartW4, 150), Qt::AlignHCenter | Qt::AlignVCenter,
				//	"��̬׼ȷ�Ȳ��Խ��");

				pPdfPainter->setPen(QPen(Qt::black, 3));

				pPdfPainter->setFont(font[3]);

				pPdfPainter->drawText(QRect(chartL4, chartT4, chartW4, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					" ��̬׼ȷ�Ȳ��Խ������λ���㣩");

				charti4++;
				pPdfPainter->drawText(QRect(chartL4, chartRowH4 * charti4 + chartT4, 278, chartRowH4 * 2), Qt::AlignHCenter | Qt::AlignVCenter,
					" ��λ ");
				pPdfPainter->drawText(QRect(chartL4 + 278, chartRowH4 * charti4 + chartT4 + 5, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					"ŷ���Ƿ���a׼ȷ��");
				pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 1, chartRowH4 * charti4 + chartT4 + 5, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					"ŷ���Ƿ���b׼ȷ��");
				pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 2, chartRowH4 * charti4 + chartT4 + 5, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					"ŷ���Ƿ���c׼ȷ��");


				charti4++;
				pPdfPainter->drawText(QRect(chartL4 + 278, chartRowH4 * charti4 + chartT4 - 5, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					"|APa|");
				pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 1, chartRowH4 * charti4 + chartT4 - 5, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					"|APb|");
				pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 2, chartRowH4 * charti4 + chartT4 - 5, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					"|APc|");


				charti4++;
				pPdfPainter->drawText(QRect(chartL4, chartRowH4 * charti4 + chartT4, 278, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					" A ");
				pPdfPainter->drawText(QRect(chartL4 + 278, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[8][0], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 1, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[8][1], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 2, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[8][2], 'f', 2)));

				charti4++;
				pPdfPainter->drawText(QRect(chartL4, chartRowH4 * charti4 + chartT4, 278, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					" B ");
				pPdfPainter->drawText(QRect(chartL4 + 278, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[9][0], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 1, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[9][1], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 2, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[9][2], 'f', 2)));

				charti4++;
				pPdfPainter->drawText(QRect(chartL4, chartRowH4 * charti4 + chartT4, 278, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					" C ");
				pPdfPainter->drawText(QRect(chartL4 + 278, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[10][0], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 1, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[10][1], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 2, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[10][2], 'f', 2)));

				charti4++;
				pPdfPainter->drawText(QRect(chartL4, chartRowH4 * charti4 + chartT4, 278, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					" D ");
				pPdfPainter->drawText(QRect(chartL4 + 278, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[11][0], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 1, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[11][1], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 2, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[11][2], 'f', 2)));

				charti4++;
				pPdfPainter->drawText(QRect(chartL4, chartRowH4 * charti4 + chartT4, 278, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					" E ");
				pPdfPainter->drawText(QRect(chartL4 + 278, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[12][0], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 1, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[12][1], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 2, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[12][2], 'f', 2)));

				charti4++;
				pPdfPainter->drawText(QRect(chartL4, chartRowH4 * charti4 + chartT4, 278, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					" F ");
				pPdfPainter->drawText(QRect(chartL4 + 278, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[13][0], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 1, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[13][1], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 2, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[13][2], 'f', 2)));

				charti4++;
				pPdfPainter->drawText(QRect(chartL4, chartRowH4 * charti4 + chartT4, 278, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					" G ");
				pPdfPainter->drawText(QRect(chartL4 + 278, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[14][0], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 1, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[14][1], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 2, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[14][2], 'f', 2)));

				charti4++;
				pPdfPainter->drawText(QRect(chartL4, chartRowH4 * charti4 + chartT4, 278, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					" H ");
				pPdfPainter->drawText(QRect(chartL4 + 278, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[15][0], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 1, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[15][1], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 2, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[15][2], 'f', 2)));

				//���ֵ
				QVector <double> data8{ array[8][0], array[9][0], array[10][0], array[11][0], array[12][0], array[13][0], array[14][0], array[15][0] };
				auto max8 = std::max_element(std::begin(data8), std::end(data8));
				double biggest8 = *max8;

				QVector <double> data9{ array[8][1], array[9][1], array[10][1], array[11][1], array[12][1], array[13][1], array[14][1], array[15][1] };
				auto max9 = std::max_element(std::begin(data9), std::end(data9));
				double biggest9 = *max9;

				QVector <double> data10{ array[8][2], array[9][2], array[10][2], array[11][2], array[12][2], array[13][2], array[14][2], array[15][2] };
				auto max10 = std::max_element(std::begin(data10), std::end(data10));
				double biggest10 = *max10;




				charti4++;
				pPdfPainter->drawText(QRect(chartL4, chartRowH4 * charti4 + chartT4, 278, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					" ���ֵ ");
				pPdfPainter->drawText(QRect(chartL4 + 278, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(biggest8, 'f', 2)));
				pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 1, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(biggest9, 'f', 2)));
				pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 2, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(biggest10, 'f', 2)));

				charti4++;
				pPdfPainter->drawText(QRect(chartL4, chartRowH4 * charti4 + chartT4, 278, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					" Ҫ�� ");
				pPdfPainter->drawText(QRect(chartL4 + 278, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					"�� 3");
				pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 1, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					"�� 3");
				pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 2, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					"�� 3");

				charti4++;
				pPdfPainter->drawText(QRect(chartL4, chartRowH4 * charti4 + chartT4, 278, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
					" �Ƿ�ͨ�� ");
				if (biggest8 <= 3)
				{
					pPdfPainter->drawText(QRect(chartL4 + 278, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
						"ͨ��");
				}

				else
				{
					pPdfPainter->drawText(QRect(chartL4 + 278, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
						"δͨ��");
				}

				if (biggest9 <= 3)
				{
					pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 1, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
						"ͨ��");
				}

				else
				{
					pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 1, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
						"δͨ��");
				}

				if (biggest10 <= 3)
				{
					pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 2, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
						"ͨ��");
				}

				else
				{
					pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 2, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
						"δͨ��");
				}





				//����
				pPdfPainter->drawLine(chartL4 + 278, chartT4 + chartRowH4, chartL4 + 278, chartT4 + chartH4);
				pPdfPainter->drawLine(chartL4 + 278 + 600 * 1, chartT4 + chartRowH4, chartL4 + 278 + 600 * 1, chartT4 + chartH4);
				pPdfPainter->drawLine(chartL4 + 278 + 600 * 2, chartT4 + chartRowH4, chartL4 + 278 + 600 * 2, chartT4 + chartH4);
				//pPdfPainter->drawLine(chartL4 + 278 + 600 * 3, chartT4 + chartRowH4, chartL4 + 278 + 600 * 3, chartT4 + chartH4);
				//pPdfPainter->drawLine(chartL4 + 278 + 450 * 4, chartT4, chartL4 + 278 + 450 * 4, chartT4 + chartH4);
				//pPdfPainter->drawLine(chartL4 + 278 + 450 * 5, chartT4, chartL4 + 278 + 450 * 5, chartT4 + chartH4);

				//����
				int chartj4 = 1;
				pPdfPainter->drawLine(chartL4, chartT4 + chartRowH4 * chartj4, chartL4 + chartW4, chartT4 + chartRowH4 * chartj4);
				chartj4++;
				chartj4++;
				pPdfPainter->drawLine(chartL4, chartT4 + chartRowH4 * chartj4, chartL4 + chartW4, chartT4 + chartRowH4 * chartj4);
				chartj4++;
				pPdfPainter->drawLine(chartL4, chartT4 + chartRowH4 * chartj4, chartL4 + chartW4, chartT4 + chartRowH4 * chartj4);
				chartj4++;
				pPdfPainter->drawLine(chartL4, chartT4 + chartRowH4 * chartj4, chartL4 + chartW4, chartT4 + chartRowH4 * chartj4);
				chartj4++;
				pPdfPainter->drawLine(chartL4, chartT4 + chartRowH4 * chartj4, chartL4 + chartW4, chartT4 + chartRowH4 * chartj4);
				chartj4++;
				pPdfPainter->drawLine(chartL4, chartT4 + chartRowH4 * chartj4, chartL4 + chartW4, chartT4 + chartRowH4 * chartj4);
				chartj4++;
				pPdfPainter->drawLine(chartL4, chartT4 + chartRowH4 * chartj4, chartL4 + chartW4, chartT4 + chartRowH4 * chartj4);
				chartj4++;
				pPdfPainter->drawLine(chartL4, chartT4 + chartRowH4 * chartj4, chartL4 + chartW4, chartT4 + chartRowH4 * chartj4);
				chartj4++;
				pPdfPainter->drawLine(chartL4, chartT4 + chartRowH4 * chartj4, chartL4 + chartW4, chartT4 + chartRowH4 * chartj4);
				chartj4++;
				pPdfPainter->drawLine(chartL4, chartT4 + chartRowH4 * chartj4, chartL4 + chartW4, chartT4 + chartRowH4 * chartj4);
				chartj4++;
				pPdfPainter->drawLine(chartL4, chartT4 + chartRowH4 * chartj4, chartL4 + chartW4, chartT4 + chartRowH4 * chartj4);
				chartj4++;
				pPdfPainter->drawLine(chartL4, chartT4 + chartRowH4 * chartj4, chartL4 + chartW4, chartT4 + chartRowH4 * chartj4);



				//��񡪡���̬�ظ��Բ��Խ��
				int chartW5 = 2478 - 400;
				int chartH5 = 980;
				int chartL5 = 200;
				int chartT5 = 1400;
				int chartRowH5 = 70;
				int charti5 = 0;
				pPdfPainter->setPen(QPen(Qt::black, 5));
				pPdfPainter->drawRect(chartL5, chartT5, chartW5, chartH5);

				//pPdfPainter->setFont(font[1]);
				//pPdfPainter->drawText(QRect(chartL5, chartT5 - 159, chartW5, 150), Qt::AlignHCenter | Qt::AlignVCenter,
				//	"��̬�ظ��Բ��Խ��");

				pPdfPainter->setPen(QPen(Qt::black, 3));

				pPdfPainter->setFont(font[3]);

				pPdfPainter->drawText(QRect(chartL5, chartT5, chartW5, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					" ��̬�ظ��Բ��Խ������λ���㣩");

				charti5++;
				pPdfPainter->drawText(QRect(chartL5, chartRowH5 * charti5 + chartT5, 278, chartRowH5 * 2), Qt::AlignHCenter | Qt::AlignVCenter,
					" ��λ ");
				pPdfPainter->drawText(QRect(chartL5 + 278, chartRowH5 * charti5 + chartT5 + 5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					"ŷ���Ƿ���a�ظ���");
				pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 1, chartRowH5 * charti5 + chartT5 + 5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					"ŷ���Ƿ���b�ظ���");
				pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 2, chartRowH5 * charti5 + chartT5 + 5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					"ŷ���Ƿ���c�ظ���");


				charti5++;
				pPdfPainter->drawText(QRect(chartL5 + 278, chartRowH5 * charti5 + chartT5 - 5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					"RPa(��)");
				pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 1, chartRowH5 * charti5 + chartT5 - 5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					"RPb(��)");
				pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 2, chartRowH5 * charti5 + chartT5 - 5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					"RPc");


				charti5++;
				pPdfPainter->drawText(QRect(chartL5, chartRowH5 * charti5 + chartT5, 278, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					" A ");
				pPdfPainter->drawText(QRect(chartL5 + 278, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[16][0], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 1, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[16][1], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 2, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[16][2], 'f', 2)));

				charti5++;
				pPdfPainter->drawText(QRect(chartL5, chartRowH5 * charti5 + chartT5, 278, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					" B ");
				pPdfPainter->drawText(QRect(chartL5 + 278, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[17][0], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 1, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[17][1], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 2, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[17][2], 'f', 2)));

				charti5++;
				pPdfPainter->drawText(QRect(chartL5, chartRowH5 * charti5 + chartT5, 278, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					" C ");
				pPdfPainter->drawText(QRect(chartL5 + 278, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[18][0], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 1, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[18][1], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 2, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[18][2], 'f', 2)));

				charti5++;
				pPdfPainter->drawText(QRect(chartL5, chartRowH5 * charti5 + chartT5, 278, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					" D ");
				pPdfPainter->drawText(QRect(chartL5 + 278, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[19][0], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 1, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[19][1], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 2, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[19][2], 'f', 2)));

				charti5++;
				pPdfPainter->drawText(QRect(chartL5, chartRowH5 * charti5 + chartT5, 278, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					" E ");
				pPdfPainter->drawText(QRect(chartL5 + 278, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[20][0], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 1, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[20][1], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 2, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[20][2], 'f', 2)));

				charti5++;
				pPdfPainter->drawText(QRect(chartL5, chartRowH5 * charti5 + chartT5, 278, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					" F ");
				pPdfPainter->drawText(QRect(chartL5 + 278, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[21][0], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 1, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[21][1], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 2, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[21][2], 'f', 2)));

				charti5++;
				pPdfPainter->drawText(QRect(chartL5, chartRowH5 * charti5 + chartT5, 278, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					" G ");
				pPdfPainter->drawText(QRect(chartL5 + 278, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[22][0], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 1, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[22][1], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 2, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[22][2], 'f', 2)));

				charti5++;
				pPdfPainter->drawText(QRect(chartL5, chartRowH5 * charti5 + chartT5, 278, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					" H ");
				pPdfPainter->drawText(QRect(chartL5 + 278, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[23][0], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 1, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[23][1], 'f', 2)));
				pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 2, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(array[23][2], 'f', 2)));

				//���ֵ
				QVector <double> data11{ array[16][0], array[17][0], array[18][0], array[19][0], array[20][0], array[21][0], array[22][0], array[23][0] };
				auto max11 = std::max_element(std::begin(data11), std::end(data11));
				double biggest11 = *max11;

				QVector <double> data12{ array[16][1], array[17][1], array[18][1], array[19][1], array[20][1], array[21][1], array[22][1], array[23][1] };
				auto max12 = std::max_element(std::begin(data12), std::end(data12));
				double biggest12 = *max12;

				QVector <double> data13{ array[16][2], array[17][2], array[18][2], array[19][2], array[20][2], array[21][2], array[22][2], array[23][2] };
				auto max13 = std::max_element(std::begin(data13), std::end(data13));
				double biggest13 = *max13;


				charti5++;
				pPdfPainter->drawText(QRect(chartL5, chartRowH5 * charti5 + chartT5, 278, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					" ���ֵ ");
				pPdfPainter->drawText(QRect(chartL5 + 278, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(biggest11, 'f', 2)));
				pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 1, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(biggest12, 'f', 2)));
				pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 2, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg(QString::number(biggest13, 'f', 2)));


				charti5++;
				pPdfPainter->drawText(QRect(chartL5, chartRowH5 * charti5 + chartT5, 278, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					" Ҫ�� ");
				pPdfPainter->drawText(QRect(chartL5 + 278, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					"�� 2");
				pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 1, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					"�� 2");
				pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 2, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					"�� 2");

				charti5++;
				pPdfPainter->drawText(QRect(chartL5, chartRowH5 * charti5 + chartT5, 278, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
					" �Ƿ�ͨ�� ");
				if (biggest11 <= 2)
				{
					pPdfPainter->drawText(QRect(chartL5 + 278, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
						"ͨ��");
				}

				else
				{
					pPdfPainter->drawText(QRect(chartL5 + 278, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
						"δͨ��");
				}

				if (biggest12 <= 2)
				{
					pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 1, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
						"ͨ��");
				}

				else
				{
					pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 1, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
						"δͨ��");
				}


				if (biggest13 <= 2)
				{
					pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 2, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
						"ͨ��");
				}

				else
				{
					pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 2, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
						"δͨ��");
				}






				//����
				pPdfPainter->drawLine(chartL5 + 278, chartT5 + chartRowH5, chartL5 + 278, chartT5 + chartH5);
				pPdfPainter->drawLine(chartL5 + 278 + 600 * 1, chartT5 + chartRowH5, chartL5 + 278 + 600 * 1, chartT5 + chartH5);
				pPdfPainter->drawLine(chartL5 + 278 + 600 * 2, chartT5 + chartRowH5, chartL5 + 278 + 600 * 2, chartT5 + chartH5);
				//pPdfPainter->drawLine(chartL5 + 278 + 600 * 3, chartT5 + chartRowH5, chartL5 + 278 + 600 * 3, chartT5 + chartH5);
				//pPdfPainter->drawLine(chartL5 + 278 + 450 * 4, chartT5, chartL5 + 278 + 450 * 4, chartT5 + chartH5);
				//pPdfPainter->drawLine(chartL5 + 278 + 450 * 5, chartT5, chartL5 + 278 + 450 * 5, chartT5 + chartH5);

				//����
				int chartj5 = 1;
				pPdfPainter->drawLine(chartL5, chartT5 + chartRowH5 * chartj5, chartL5 + chartW5, chartT5 + chartRowH5 * chartj5);
				chartj5++;
				chartj5++;
				pPdfPainter->drawLine(chartL5, chartT5 + chartRowH5 * chartj5, chartL5 + chartW5, chartT5 + chartRowH5 * chartj5);
				chartj5++;
				pPdfPainter->drawLine(chartL5, chartT5 + chartRowH5 * chartj5, chartL5 + chartW5, chartT5 + chartRowH5 * chartj5);
				chartj5++;
				pPdfPainter->drawLine(chartL5, chartT5 + chartRowH5 * chartj5, chartL5 + chartW5, chartT5 + chartRowH5 * chartj5);
				chartj5++;
				pPdfPainter->drawLine(chartL5, chartT5 + chartRowH5 * chartj5, chartL5 + chartW5, chartT5 + chartRowH5 * chartj5);
				chartj5++;
				pPdfPainter->drawLine(chartL5, chartT5 + chartRowH5 * chartj5, chartL5 + chartW5, chartT5 + chartRowH5 * chartj5);
				chartj5++;
				pPdfPainter->drawLine(chartL5, chartT5 + chartRowH5 * chartj5, chartL5 + chartW5, chartT5 + chartRowH5 * chartj5);
				chartj5++;
				pPdfPainter->drawLine(chartL5, chartT5 + chartRowH5 * chartj5, chartL5 + chartW5, chartT5 + chartRowH5 * chartj5);
				chartj5++;
				pPdfPainter->drawLine(chartL5, chartT5 + chartRowH5 * chartj5, chartL5 + chartW5, chartT5 + chartRowH5 * chartj5);
				chartj5++;
				pPdfPainter->drawLine(chartL5, chartT5 + chartRowH5 * chartj5, chartL5 + chartW5, chartT5 + chartRowH5 * chartj5);
				chartj5++;
				pPdfPainter->drawLine(chartL5, chartT5 + chartRowH5 * chartj5, chartL5 + chartW5, chartT5 + chartRowH5 * chartj5);
				chartj5++;
				pPdfPainter->drawLine(chartL5, chartT5 + chartRowH5 * chartj5, chartL5 + chartW5, chartT5 + chartRowH5 * chartj5);





				//ҳ��
				pPdfPainter->setFont(font[3]);
				pPdfPainter->setPen(QPen(Qt::darkGray, 1));
				pPdfPainter->drawLine(150, 3310, 2328, 3310);  //2478-150

				pPdfPainter->drawText(QRect(1100, 3200, 300, 300), Qt::AlignHCenter | Qt::AlignVCenter,
					"2/2");
				pPdfPainter->drawText(QRect(1820, 3200, 570, 300), Qt::AlignHCenter | Qt::AlignVCenter,
					"������Ϣ �ϸ���");
				pPdfPainter->drawText(QRect(50, 3200, 700, 300), Qt::AlignHCenter | Qt::AlignVCenter,
					QString("%1").arg("SR-ENS-600"));
				pPdfPainter->setFont(font[2]);




				delete pPdfPainter;
				delete pPdfWriter;
				pdfFile.close();
				//������ǰ·������Ϊԭ����·��


				tempDir.setCurrent(currentDir);
				delete m_pProgressBar;
				QMessageBox::warning(this, "��ʾ", "�������");
				setTipWindow("���Ӳ���׼ȷ�Ⱥ��ظ��Բ��Ա������ɳɹ���");
	}
}

void GenRep::CreatePdfSensitivitySlot()
{
	setTipWindow("���ӿ��������ȵ��ڲ��Ա�����������...");
	QString str = file_path +"result_SCALE32.log";
	QFile file1(str);
	if (!file1.open(QIODevice::ReadOnly | QIODevice::Text)) 
	{
		setTipWindow("���ӿ��������ȵ��ڲ��Ա�������ʧ��");
		return;
	}

	str = file_path + "result_SCALE21.log";
	QFile file2(str);
	if (!file2.open(QIODevice::ReadOnly | QIODevice::Text)) 
	{
		setTipWindow("���ӿ��������ȵ��ڲ��Ա�������ʧ��");
		return;
	}

	str = file_path + "result_SCALE31.log";
	QFile file3(str);
	if (!file3.open(QIODevice::ReadOnly | QIODevice::Text)) 
	{
		setTipWindow("���ӿ��������ȵ��ڲ��Ա�������ʧ��");
		return;
	}
	//�������ǰ·��
	QString currentDir = QDir::currentPath();
	//����pdf�����·��
	QString file_path = QFileDialog::getExistingDirectory(this, "��ѡ�񱣴�·��", "C:\\Users\\Administrator\\Desktop");
	if (file_path.isEmpty())
	{
		setTipWindow("���ӿ��������ȵ��ڲ��Ա�������ʧ��");
		QMessageBox::warning(this, "��ʾ", "·������Ϊ��");
		return;
		
	}

	//�������ִ��·�����õ�filePath��
	QDir tempDir;
	tempDir.setCurrent(file_path);

	//������
	QProgressBar* m_pProgressBar = new QProgressBar(this);
	m_pProgressBar->setOrientation(Qt::Horizontal);  // ˮƽ����
	m_pProgressBar->setMinimum(1);  // ��Сֵ
	m_pProgressBar->setMaximum(1);  // ���ֵ str.size()
	m_pProgressBar->setMaximumHeight(15);
	m_pProgressBar->setMinimumWidth(60);
	m_pProgressBar->setVisible(true);


	m_pProgressBar->setValue(0);
	QCoreApplication::processEvents();

	QDate date = QDate::currentDate();

	//QString serialNum = QString::number(Production.at(ui.toolComboBox->currentIndex()).second);
	//QString year = str.at(3).left(4);

	QString fileName = QString::fromStdString("���ӿ��������ȵ��ڲ���") + ".pdf";
	QFile pdfFile(fileName);
	//�ж��ļ��Ƿ����
	if (QFile::exists(fileName))
	{
		QMessageBox::StandardButton reply;
		reply = QMessageBox::question(this, "��ʾ", "�ļ��Ѿ�����,����ǽ�����ԭ�ļ�", QMessageBox::Yes | QMessageBox::No);
		if (reply == QMessageBox::Yes)
		{
			QFile::remove(fileName);
		}
		else
		{
			setTipWindow("���ӿ��������ȵ��ڲ��Ա�������ʧ��");
			return;
		}
	}
	if (1)
	{
		// ��Ҫд���pdf�ļ�
		pdfFile.open(QIODevice::WriteOnly);

		QPdfWriter* pPdfWriter = new QPdfWriter(&pdfFile);  // ����pdfд����
		pPdfWriter->setPageSize(QPagedPaintDevice::A4);     // ����ֽ��ΪA4
		pPdfWriter->setResolution(300);                     // ����ֽ�ŵķֱ���Ϊ300,���������Ϊ3508X2479

		int iMargin = 0;                   // ҳ�߾�
		pPdfWriter->setPageMargins(QMarginsF(iMargin, iMargin, iMargin, iMargin));

		QPainter* pPdfPainter = new QPainter(pPdfWriter);   // qt���ƹ���

		// ����,����
		QTextOption option(Qt::AlignHCenter | Qt::AlignVCenter);
		option.setWrapMode(QTextOption::WordWrap);

		//����
		//QFont font;
		//font.setFamily("simhei.ttf");
		QFont font[8] = { QFont("����", 26, 60), QFont("����", 26, 61), QFont("����", 26, QFont::Normal), QFont("times new roman", 26, QFont::Normal), QFont("����", 26, QFont::Normal), QFont("������κ", 26, 61), QFont("����", 26, QFont::Normal), QFont("times new roman", 26, QFont::Normal) };
		font[0].setPixelSize(86);
		font[1].setPixelSize(55);
		font[2].setPixelSize(40);
		font[3].setPixelSize(40);
		font[4].setPixelSize(50);
		font[5].setPixelSize(50);
		font[6].setPixelSize(40);
		font[7].setPixelSize(50);
		//Painter PDF
		qDebug() << pPdfPainter->viewport();
		int nPDFWidth = pPdfPainter->viewport().width();
		int nPDFHeight = pPdfPainter->viewport().height();

		//pPdfPainter->setPen(QPen(Qt::black, 5));
		//pPdfPainter->drawLine(0, 0, 2478, 0);
		//pPdfPainter->size
		//����ͼƬˮƽ�߾�Ϊ150
		//pPdfPainter->setOpacity(0.4);
		//pPdfPainter->drawPixmap(5000, 100, 220 * 1.3, 110 * 1.3, pixmap);
		//pPdfPainter->drawPixmap(200, 100, 220 * 1.3, 110 * 1.3, pixmap);
		//pPdfPainter->setOpacity(1.0);


		//ͼֽ��С
		int pdfWidth = 2478;
		int pdfHeight = 3508;
		pPdfPainter->setFont(font[2]);
		pPdfPainter->setPen(QPen(Qt::darkGray, 1));




		pPdfPainter->drawLine(150, 270, 2328, 270);  //2478-150

		pPdfPainter->setFont(font[5]);
		pPdfPainter->setPen(QPen(Qt::black, 2));
		pPdfPainter->drawText(QRect(0, 0, 2478 / 3, 250), Qt::AlignHCenter | Qt::AlignBottom,
			"�������������޹�˾");


		//���Ʊ�����Ϣ
		int chartW1 = 2478 - 400;
		int chartH1 = 950;
		int chartL1 = 200;
		int chartT1 = 550;
		int chartRowH1 = 85;
		int charti1 = 0;
		pPdfPainter->setFont(font[1]);
		pPdfPainter->setPen(QPen(Qt::black, 3));

		pPdfPainter->drawText(QRect(150, 300, 2328, 200), Qt::AlignHCenter | Qt::AlignVCenter,
			QString("%1����").arg("���ӿ��������ȵ��ڲ���"));

		pPdfPainter->setFont(font[4]);
		pPdfPainter->setPen(QPen(Qt::black, 5));
		pPdfPainter->drawRect(chartL1, chartT1, chartW1, chartH1);
		pPdfPainter->setPen(QPen(Qt::black));
		pPdfPainter->setFont(font[2]);
		chartL1 += 15;
		pPdfPainter->drawText(QRect(chartL1 + 90, chartRowH1 * -1 + chartT1 + 12, pdfWidth / 4, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "���� ��9527");
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 340, chartRowH1 * -1 + chartT1 + 12, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "��� ��         ");

		pPdfPainter->setFont(font[6]);
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("            %1").arg("�ڿ�������ϵͳ"));

		pPdfPainter->setFont(font[2]);
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 260, chartRowH1 * charti1 + chartT1 + 10, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "�� �� Ա ��         ");
		charti1++;
		pPdfPainter->drawLine(pdfWidth / 2 + 500, chartRowH1 * charti1 + chartT1 + 5, chartW1 + 100, chartRowH1 * charti1 + chartT1 + 5);

		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("��Ʒ�ͺ� ��%1").arg(system_proc));
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 260, chartRowH1 * charti1 + chartT1 + 10, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "�������� ��         ");
		charti1++;
		pPdfPainter->drawLine(pdfWidth / 2 + 500, chartRowH1 * charti1 + chartT1 + 5, chartW1 + 100, chartRowH1 * charti1 + chartT1 + 5);
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�� �� �� ��%1").arg(system_serial));
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 260, chartRowH1 * charti1 + chartT1 + 10, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "�� �� Ա ��         ");
		charti1++;
		pPdfPainter->drawLine(pdfWidth / 2 + 500, chartRowH1 * charti1 + chartT1 + 5, chartW1 + 100, chartRowH1 * charti1 + chartT1 + 5);
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�������� ��%1").arg(system_batch));
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 260, chartRowH1 * charti1 + chartT1 + 10, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "������� ��         ");
		charti1++;
		pPdfPainter->drawLine(pdfWidth / 2 + 500, chartRowH1 * charti1 + chartT1 + 5, chartW1 + 100, chartRowH1 * charti1 + chartT1 + 5);
		//pPdfPainter->drawLine(chartL1 + 200, chartRowH1*charti1 + chartT1 + 10, chartW1 + 100, chartRowH1*charti1 + chartT1 + 10);


		pPdfPainter->setFont(font[6]);
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("            %1").arg("˫����ͷץǯ"));
		pPdfPainter->setFont(font[2]);
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("��Ʒ�ͺ� ��%1").arg(tool_proc));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�� �� �� ��%1").arg(tool_serial));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�������� ��%1").arg(tool_batch));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�豸���� ��%1").arg(QString::fromStdString("���ӿ��������ȵ��ڲ��Թ�װ")));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�豸��� ��%1").arg(QString::fromStdString(" ")));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�����׼ ��%1").arg(QString::fromStdString("������ SR-ENS-600 ����Ҫ��")));
		//pPdfPainter->drawLine(pdfWidth / 2 - 160, chartRowH1*charti1 + chartT1 + 10, chartW1 - 100, chartRowH1*charti1 + chartT1 + 10);
		pPdfPainter->setFont(font[2]);


		//��log32
		qreal array1[3][3] = { 0 };



		QTextStream in1(&file1);

		int line_number1 = 0;
		while (!in1.atEnd()) {
			QString line1 = in1.readLine();
			QStringList str_list1 = line1.split(QRegExp(","));
			for (int i1 = 0; i1 < str_list1.length(); ++i1) {
				array1[line_number1][i1] = str_list1[i1].toFloat();
			}
			++line_number1;
		}
		file1.close();



		//��񡪡�ӳ���3:2
		int chartW2 = 2478 - 400;
		int chartH2 = 420;
		int chartL2 = 200;
		int chartT2 = 1650;
		int chartRowH2 = 70;
		int charti2 = 0;
		pPdfPainter->setPen(QPen(Qt::black, 5));
		pPdfPainter->drawRect(chartL2, chartT2, chartW2, chartH2);



		pPdfPainter->setPen(QPen(Qt::black, 3));
		pPdfPainter->setFont(font[3]);

		//����
		pPdfPainter->drawText(QRect(chartL2, chartT2 - 100, chartW2, 100), Qt::AlignHCenter | Qt::AlignVCenter,
			"�� 1 �����Ȳ��Խ����¼��ӳ���3:2��");


		//����
		pPdfPainter->drawLine(chartL2 + 578 + 500 * 0, chartT2, chartL2 + 578 + 500 * 0, chartT2 + chartH2);
		pPdfPainter->drawLine(chartL2 + 578 + 500 * 1, chartT2, chartL2 + 578 + 500 * 1, chartT2 + chartRowH2 * 4);
		pPdfPainter->drawLine(chartL2 + 578 + 500 * 2, chartT2, chartL2 + 578 + 500 * 2, chartT2 + chartRowH2 * 4);


		//����
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 1, chartL2 + chartW2, chartT2 + chartRowH2 * 1);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 2, chartL2 + chartW2, chartT2 + chartRowH2 * 2);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 3, chartL2 + chartW2, chartT2 + chartRowH2 * 3);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 4, chartL2 + chartW2, chartT2 + chartRowH2 * 4);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 5, chartL2 + chartW2, chartT2 + chartRowH2 * 5);

		pPdfPainter->setFont(font[3]);


		//�������

		pPdfPainter->drawText(QRect(chartL2, chartT2, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"ӳ���(3:2)");
		pPdfPainter->drawText(QRect(chartL2 + 578 + 500 * 0, chartT2, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"X");
		pPdfPainter->drawText(QRect(chartL2 + 578 + 500 * 1, chartT2, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"Y");
		pPdfPainter->drawText(QRect(chartL2 + 578 + 500 * 2, chartT2, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"Z");

		charti2++;

		pPdfPainter->drawText(QRect(chartL2, chartT2 + chartRowH2 * charti2, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"1");
		pPdfPainter->drawText(QRect(chartL2 + 578 + 500 * 0, chartT2 + chartRowH2 * charti2, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array1[0][0], 'f', 5)));
		pPdfPainter->drawText(QRect(chartL2 + 578 + 500 * 1, chartT2 + chartRowH2 * charti2, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array1[0][1], 'f', 5)));
		pPdfPainter->drawText(QRect(chartL2 + 578 + 500 * 2, chartT2 + chartRowH2 * charti2, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array1[0][2], 'f', 5)));

		charti2++;

		pPdfPainter->drawText(QRect(chartL2, chartT2 + chartRowH2 * charti2, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"2");
		pPdfPainter->drawText(QRect(chartL2 + 578 + 500 * 0, chartT2 + chartRowH2 * charti2, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array1[1][0], 'f', 5)));
		pPdfPainter->drawText(QRect(chartL2 + 578 + 500 * 1, chartT2 + chartRowH2 * charti2, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array1[1][1], 'f', 5)));
		pPdfPainter->drawText(QRect(chartL2 + 578 + 500 * 2, chartT2 + chartRowH2 * charti2, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array1[1][2], 'f', 5)));

		charti2++;

		pPdfPainter->drawText(QRect(chartL2, chartT2 + chartRowH2 * charti2, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"2");
		pPdfPainter->drawText(QRect(chartL2 + 578 + 500 * 0, chartT2 + chartRowH2 * charti2, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array1[2][0], 'f', 5)));
		pPdfPainter->drawText(QRect(chartL2 + 578 + 500 * 1, chartT2 + chartRowH2 * charti2, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array1[2][1], 'f', 5)));
		pPdfPainter->drawText(QRect(chartL2 + 578 + 500 * 2, chartT2 + chartRowH2 * charti2, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array1[2][2], 'f', 5)));

		charti2++;

		pPdfPainter->drawText(QRect(chartL2, chartT2 + chartRowH2 * charti2, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"��׼ֵ");
		pPdfPainter->drawText(QRect(chartL2 + 578 + 500 * 0, chartT2 + chartRowH2 * charti2, 1500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			" 3��2  �� 5%");

		charti2++;

		pPdfPainter->drawText(QRect(chartL2, chartT2 + chartRowH2 * charti2, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"����");


		if ((array1[0][0] >= 5) || (array1[0][1] >= 5) || (array1[0][2] >= 5) || (array1[1][0] >= 5) || (array1[1][1] >= 5) || (array1[1][2] >= 5) || (array1[2][0] >= 5) || (array1[2][1] >= 5) || (array1[2][2] >= 5))
		{
			pPdfPainter->drawText(QRect(chartL2 + 578 + 500 * 0, chartT2 + chartRowH2 * charti2, 1500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				" ���ϸ�");
		}
		else if ((array1[0][0] <= -5) || (array1[0][1] <= -5) || (array1[0][2] <= -5) || (array1[1][0] <= -5) || (array1[1][1] <= -5) || (array1[1][2] <= -5) || (array1[2][0] <= -5) || (array1[2][1] <= -5) || (array1[2][2] <= -5))
			pPdfPainter->drawText(QRect(chartL2 + 578 + 500 * 0, chartT2 + chartRowH2 * charti2, 1500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				" ���ϸ�");
		else
			pPdfPainter->drawText(QRect(chartL2 + 578 + 500 * 0, chartT2 + chartRowH2 * charti2, 1500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				" �ϸ�");



		//��log21
		qreal array2[3][3] = { 0 };



		QTextStream in2(&file2);

		int line_number2 = 0;
		while (!in2.atEnd()) {
			QString line2 = in2.readLine();
			QStringList str_list2 = line2.split(QRegExp(","));
			for (int i2 = 0; i2 < str_list2.length(); ++i2) {
				array2[line_number2][i2] = str_list2[i2].toFloat();
			}
			++line_number2;
		}
		file2.close();

		//��񡪡�ӳ���2:1
		int chartW3 = 2478 - 400;
		int chartH3 = 420;
		int chartL3 = 200;
		int chartT3 = 2200;
		int chartRowH3 = 70;
		int charti3 = 0;
		pPdfPainter->setPen(QPen(Qt::black, 5));
		pPdfPainter->drawRect(chartL3, chartT3, chartW3, chartH3);



		pPdfPainter->setPen(QPen(Qt::black, 3));
		pPdfPainter->setFont(font[3]);

		//����
		pPdfPainter->drawText(QRect(chartL3, chartT3 - 100, chartW3, 100), Qt::AlignHCenter | Qt::AlignVCenter,
			"�� 2 �����Ȳ��Խ����¼��ӳ���2:1��");


		//����
		pPdfPainter->drawLine(chartL3 + 578 + 500 * 0, chartT3, chartL3 + 578 + 500 * 0, chartT3 + chartH3);
		pPdfPainter->drawLine(chartL3 + 578 + 500 * 1, chartT3, chartL3 + 578 + 500 * 1, chartT3 + chartRowH3 * 4);
		pPdfPainter->drawLine(chartL3 + 578 + 500 * 2, chartT3, chartL3 + 578 + 500 * 2, chartT3 + chartRowH3 * 4);


		//����
		pPdfPainter->drawLine(chartL3, chartT3 + chartRowH3 * 1, chartL3 + chartW3, chartT3 + chartRowH3 * 1);
		pPdfPainter->drawLine(chartL3, chartT3 + chartRowH3 * 2, chartL3 + chartW3, chartT3 + chartRowH3 * 2);
		pPdfPainter->drawLine(chartL3, chartT3 + chartRowH3 * 3, chartL3 + chartW3, chartT3 + chartRowH3 * 3);
		pPdfPainter->drawLine(chartL3, chartT3 + chartRowH3 * 4, chartL3 + chartW3, chartT3 + chartRowH3 * 4);
		pPdfPainter->drawLine(chartL3, chartT3 + chartRowH3 * 5, chartL3 + chartW3, chartT3 + chartRowH3 * 5);

		pPdfPainter->setFont(font[3]);


		//�������

		pPdfPainter->drawText(QRect(chartL3, chartT3, 578, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			"ӳ���(2:1)");
		pPdfPainter->drawText(QRect(chartL3 + 578 + 500 * 0, chartT3, 500, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			"X");
		pPdfPainter->drawText(QRect(chartL3 + 578 + 500 * 1, chartT3, 500, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			"Y");
		pPdfPainter->drawText(QRect(chartL3 + 578 + 500 * 2, chartT3, 500, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			"Z");

		charti3++;

		pPdfPainter->drawText(QRect(chartL3, chartT3 + chartRowH3 * charti3, 578, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			"1");
		pPdfPainter->drawText(QRect(chartL3 + 578 + 500 * 0, chartT3 + chartRowH3 * charti3, 500, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array2[0][0], 'f', 5)));
		pPdfPainter->drawText(QRect(chartL3 + 578 + 500 * 1, chartT3 + chartRowH3 * charti3, 500, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array2[0][1], 'f', 5)));
		pPdfPainter->drawText(QRect(chartL3 + 578 + 500 * 2, chartT3 + chartRowH3 * charti3, 500, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array2[0][2], 'f', 5)));

		charti3++;

		pPdfPainter->drawText(QRect(chartL3, chartT3 + chartRowH3 * charti3, 578, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			"2");
		pPdfPainter->drawText(QRect(chartL3 + 578 + 500 * 0, chartT3 + chartRowH3 * charti3, 500, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array2[1][0], 'f', 5)));
		pPdfPainter->drawText(QRect(chartL3 + 578 + 500 * 1, chartT3 + chartRowH3 * charti3, 500, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array2[1][1], 'f', 5)));
		pPdfPainter->drawText(QRect(chartL3 + 578 + 500 * 2, chartT3 + chartRowH3 * charti3, 500, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array2[1][2], 'f', 5)));

		charti3++;

		pPdfPainter->drawText(QRect(chartL3, chartT3 + chartRowH3 * charti3, 578, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			"2");
		pPdfPainter->drawText(QRect(chartL3 + 578 + 500 * 0, chartT3 + chartRowH3 * charti3, 500, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array2[2][0], 'f', 5)));
		pPdfPainter->drawText(QRect(chartL3 + 578 + 500 * 1, chartT3 + chartRowH3 * charti3, 500, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array2[2][1], 'f', 5)));
		pPdfPainter->drawText(QRect(chartL3 + 578 + 500 * 2, chartT3 + chartRowH3 * charti3, 500, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array2[2][2], 'f', 5)));

		charti3++;

		pPdfPainter->drawText(QRect(chartL3, chartT3 + chartRowH3 * charti3, 578, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			"��׼ֵ");
		pPdfPainter->drawText(QRect(chartL3 + 578 + 500 * 0, chartT3 + chartRowH3 * charti3, 1500, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			" 2��1  �� 5%");

		charti3++;

		pPdfPainter->drawText(QRect(chartL3, chartT3 + chartRowH3 * charti3, 578, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
			"����");


		if ((array2[0][0] >= 5) || (array2[0][1] >= 5) || (array2[0][2] >= 5) || (array2[1][0] >= 5) || (array2[1][1] >= 5) || (array2[1][2] >= 5) || (array2[2][0] >= 5) || (array2[2][1] >= 5) || (array2[2][2] >= 5))
		{
			pPdfPainter->drawText(QRect(chartL3 + 578 + 500 * 0, chartT3 + chartRowH3 * charti3, 1500, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				" ���ϸ�");
		}
		else if ((array2[0][0] <= -5) || (array2[0][1] <= -5) || (array2[0][2] <= -5) || (array2[1][0] <= -5) || (array2[1][1] <= -5) || (array2[1][2] <= -5) || (array2[2][0] <= -5) || (array2[2][1] <= -5) || (array2[2][2] <= -5))
			pPdfPainter->drawText(QRect(chartL3 + 578 + 500 * 0, chartT3 + chartRowH3 * charti3, 1500, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				" ���ϸ�");
		else
			pPdfPainter->drawText(QRect(chartL3 + 578 + 500 * 0, chartT3 + chartRowH3 * charti3, 1500, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				" �ϸ�");


		//��log31
		qreal array3[3][3] = { 0 };


		QTextStream in3(&file3);

		int line_number3 = 0;
		while (!in3.atEnd()) {
			QString line3 = in3.readLine();
			QStringList str_list3 = line3.split(QRegExp(","));
			for (int i3 = 0; i3 < str_list3.length(); ++i3) {
				array3[line_number3][i3] = str_list3[i3].toFloat();
			}
			++line_number3;
		}
		file3.close();

		//��񡪡�ӳ���2:1
		int chartW4 = 2478 - 400;
		int chartH4 = 420;
		int chartL4 = 200;
		int chartT4 = 2750;
		int chartRowH4 = 70;
		int charti4 = 0;
		pPdfPainter->setPen(QPen(Qt::black, 5));
		pPdfPainter->drawRect(chartL4, chartT4, chartW4, chartH4);



		pPdfPainter->setPen(QPen(Qt::black, 3));
		pPdfPainter->setFont(font[3]);

		//����
		pPdfPainter->drawText(QRect(chartL4, chartT4 - 100, chartW4, 100), Qt::AlignHCenter | Qt::AlignVCenter,
			"�� 3 �����Ȳ��Խ����¼��ӳ���3:1��");


		//����
		pPdfPainter->drawLine(chartL4 + 578 + 500 * 0, chartT4, chartL4 + 578 + 500 * 0, chartT4 + chartH4);
		pPdfPainter->drawLine(chartL4 + 578 + 500 * 1, chartT4, chartL4 + 578 + 500 * 1, chartT4 + chartRowH4 * 4);
		pPdfPainter->drawLine(chartL4 + 578 + 500 * 2, chartT4, chartL4 + 578 + 500 * 2, chartT4 + chartRowH4 * 4);


		//����
		pPdfPainter->drawLine(chartL4, chartT4 + chartRowH4 * 1, chartL4 + chartW4, chartT4 + chartRowH4 * 1);
		pPdfPainter->drawLine(chartL4, chartT4 + chartRowH4 * 2, chartL4 + chartW4, chartT4 + chartRowH4 * 2);
		pPdfPainter->drawLine(chartL4, chartT4 + chartRowH4 * 3, chartL4 + chartW4, chartT4 + chartRowH4 * 3);
		pPdfPainter->drawLine(chartL4, chartT4 + chartRowH4 * 4, chartL4 + chartW4, chartT4 + chartRowH4 * 4);
		pPdfPainter->drawLine(chartL4, chartT4 + chartRowH4 * 5, chartL4 + chartW4, chartT4 + chartRowH4 * 5);

		pPdfPainter->setFont(font[3]);


		//�������

		pPdfPainter->drawText(QRect(chartL4, chartT4, 578, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
			"ӳ���(3:1)");
		pPdfPainter->drawText(QRect(chartL4 + 578 + 500 * 0, chartT4, 500, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
			"X");
		pPdfPainter->drawText(QRect(chartL4 + 578 + 500 * 1, chartT4, 500, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
			"Y");
		pPdfPainter->drawText(QRect(chartL4 + 578 + 500 * 2, chartT4, 500, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
			"Z");

		charti4++;

		pPdfPainter->drawText(QRect(chartL4, chartT4 + chartRowH4 * charti4, 578, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
			"1");
		pPdfPainter->drawText(QRect(chartL4 + 578 + 500 * 0, chartT4 + chartRowH4 * charti4, 500, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array3[0][0], 'f', 5)));
		pPdfPainter->drawText(QRect(chartL4 + 578 + 500 * 1, chartT4 + chartRowH4 * charti4, 500, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array3[0][1], 'f', 5)));
		pPdfPainter->drawText(QRect(chartL4 + 578 + 500 * 2, chartT4 + chartRowH4 * charti4, 500, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array3[0][2], 'f', 5)));

		charti4++;

		pPdfPainter->drawText(QRect(chartL4, chartT4 + chartRowH4 * charti4, 578, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
			"2");
		pPdfPainter->drawText(QRect(chartL4 + 578 + 500 * 0, chartT4 + chartRowH4 * charti4, 500, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array3[1][0], 'f', 5)));
		pPdfPainter->drawText(QRect(chartL4 + 578 + 500 * 1, chartT4 + chartRowH4 * charti4, 500, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array3[1][1], 'f', 5)));
		pPdfPainter->drawText(QRect(chartL4 + 578 + 500 * 2, chartT4 + chartRowH4 * charti4, 500, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array3[1][2], 'f', 5)));

		charti4++;

		pPdfPainter->drawText(QRect(chartL4, chartT4 + chartRowH4 * charti4, 578, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
			"2");
		pPdfPainter->drawText(QRect(chartL4 + 578 + 500 * 0, chartT4 + chartRowH4 * charti4, 500, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array3[2][0], 'f', 5)));
		pPdfPainter->drawText(QRect(chartL4 + 578 + 500 * 1, chartT4 + chartRowH4 * charti4, 500, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array3[2][1], 'f', 5)));
		pPdfPainter->drawText(QRect(chartL4 + 578 + 500 * 2, chartT4 + chartRowH4 * charti4, 500, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 %").arg(QString::number(array3[2][2], 'f', 5)));

		charti4++;

		pPdfPainter->drawText(QRect(chartL4, chartT4 + chartRowH4 * charti4, 578, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
			"��׼ֵ");
		pPdfPainter->drawText(QRect(chartL4 + 578 + 500 * 0, chartT4 + chartRowH4 * charti4, 1500, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
			" 3��1  �� 5%");

		charti4++;

		pPdfPainter->drawText(QRect(chartL4, chartT4 + chartRowH4 * charti4, 578, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
			"����");


		if ((array3[0][0] >= 5) || (array3[0][1] >= 5) || (array3[0][2] >= 5) || (array3[1][0] >= 5) || (array3[1][1] >= 5) || (array3[1][2] >= 5) || (array3[2][0] >= 5) || (array3[2][1] >= 5) || (array3[2][2] >= 5))
		{
			pPdfPainter->drawText(QRect(chartL4 + 578 + 500 * 0, chartT4 + chartRowH4 * charti4, 1500, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
				" ���ϸ�");
		}
		else if ((array3[0][0] <= -5) || (array3[0][1] <= -5) || (array3[0][2] <= -5) || (array3[1][0] <= -5) || (array3[1][1] <= -5) || (array3[1][2] <= -5) || (array3[2][0] <= -5) || (array3[2][1] <= -5) || (array3[2][2] <= -5))
			pPdfPainter->drawText(QRect(chartL4 + 578 + 500 * 0, chartT4 + chartRowH4 * charti4, 1500, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
				" ���ϸ�");
		else
			pPdfPainter->drawText(QRect(chartL4 + 578 + 500 * 0, chartT4 + chartRowH4 * charti4, 1500, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
				" �ϸ�");





		//ҳ��
		pPdfPainter->setFont(font[3]);
		pPdfPainter->setPen(QPen(Qt::darkGray, 1));
		pPdfPainter->drawLine(150, 3310, 2328, 3310);  //2478-150

		pPdfPainter->drawText(QRect(1100, 3200, 300, 300), Qt::AlignHCenter | Qt::AlignVCenter,
			"1");
		pPdfPainter->drawText(QRect(1820, 3200, 570, 300), Qt::AlignHCenter | Qt::AlignVCenter,
			"������Ϣ �ϸ���");
		pPdfPainter->drawText(QRect(50, 3200, 700, 300), Qt::AlignHCenter | Qt::AlignVCenter,
			QString("%1").arg("SR-ENS-600"));
		pPdfPainter->setFont(font[2]);

		delete pPdfPainter;
		delete pPdfWriter;
		pdfFile.close();
		//������ǰ·������Ϊԭ����·��


		tempDir.setCurrent(currentDir);
		delete m_pProgressBar;
		QMessageBox::warning(this, "��ʾ", "�������");
		setTipWindow("���ӿ��������ȵ��ڲ��Ա������ɳɹ���");
	}
}

void GenRep::CreatePdfPositionRepeatSlot()
{
	setTipWindow("λ��׼ȷ�Ⱥ��ظ��Բ��Ա�����������...");
	QString str = file_path + "pose_acc.log";
	QFile file(str);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) 
	{
		setTipWindow("λ��׼ȷ�Ⱥ��ظ��Բ��Ա�������ʧ��");
		return;
	}
	//�������ǰ·��
	QString currentDir = QDir::currentPath();
	//����pdf�����·��
	QString file_path = QFileDialog::getExistingDirectory(this, "��ѡ�񱣴�·��", "C:\\Users\\Administrator\\Desktop");
	if (file_path.isEmpty())
	{
		setTipWindow("λ��׼ȷ�Ⱥ��ظ��Բ��Ա�������ʧ��");
		QMessageBox::warning(this, "��ʾ", "·������Ϊ��");
		return;
		
	}

	//�������ִ��·�����õ�filePath��
	QDir tempDir;
	tempDir.setCurrent(file_path);

	//������
	QProgressBar* m_pProgressBar = new QProgressBar(this);
	m_pProgressBar->setOrientation(Qt::Horizontal);  // ˮƽ����
	m_pProgressBar->setMinimum(1);  // ��Сֵ
	m_pProgressBar->setMaximum(1);  // ���ֵ str.size()
	m_pProgressBar->setMaximumHeight(15);
	m_pProgressBar->setMinimumWidth(60);
	m_pProgressBar->setVisible(true);


	m_pProgressBar->setValue(0);
	QCoreApplication::processEvents();

	QDate date = QDate::currentDate();

	//QString serialNum = QString::number(Production.at(ui.toolComboBox->currentIndex()).second);
	//QString year = str.at(3).left(4);

	QString fileName = QString::fromStdString("λ��׼ȷ�Ⱥ��ظ��Բ���") + ".pdf";
	QFile pdfFile(fileName);
	//�ж��ļ��Ƿ����
	if (QFile::exists(fileName))
	{
		QMessageBox::StandardButton reply;
		reply = QMessageBox::question(this, "��ʾ", "�ļ��Ѿ�����,����ǽ�����ԭ�ļ�", QMessageBox::Yes | QMessageBox::No);
		if (reply == QMessageBox::Yes)
		{
			QFile::remove(fileName);
		}
		else
		{
			setTipWindow("λ��׼ȷ�Ⱥ��ظ��Բ��Ա�������ʧ��");
			return;
		}
	}
	if (1)
	{
		// ��Ҫд���pdf�ļ�
		pdfFile.open(QIODevice::WriteOnly);

		QPdfWriter* pPdfWriter = new QPdfWriter(&pdfFile);  // ����pdfд����
		pPdfWriter->setPageSize(QPagedPaintDevice::A4);     // ����ֽ��ΪA4
		pPdfWriter->setResolution(300);                     // ����ֽ�ŵķֱ���Ϊ300,���������Ϊ3508X2479

		int iMargin = 0;                   // ҳ�߾�
		pPdfWriter->setPageMargins(QMarginsF(iMargin, iMargin, iMargin, iMargin));

		QPainter* pPdfPainter = new QPainter(pPdfWriter);   // qt���ƹ���

		// ����,����
		QTextOption option(Qt::AlignHCenter | Qt::AlignVCenter);
		option.setWrapMode(QTextOption::WordWrap);

		//����
		//QFont font;
		//font.setFamily("simhei.ttf");
		QFont font[8] = { QFont("����", 26, 60), QFont("����", 26, 61), QFont("����", 26, QFont::Normal), QFont("times new roman", 26, QFont::Normal), QFont("����", 26, QFont::Normal), QFont("������κ", 26, 61), QFont("����", 26, QFont::Normal), QFont("times new roman", 26, QFont::Normal) };
		font[0].setPixelSize(86);
		font[1].setPixelSize(55);
		font[2].setPixelSize(40);
		font[3].setPixelSize(40);
		font[4].setPixelSize(50);
		font[5].setPixelSize(50);
		font[6].setPixelSize(40);
		font[7].setPixelSize(50);
		//Painter PDF
		qDebug() << pPdfPainter->viewport();
		int nPDFWidth = pPdfPainter->viewport().width();
		int nPDFHeight = pPdfPainter->viewport().height();

		//pPdfPainter->setPen(QPen(Qt::black, 5));
		//pPdfPainter->drawLine(0, 0, 2478, 0);
		//pPdfPainter->size
		//����ͼƬˮƽ�߾�Ϊ150
		//pPdfPainter->setOpacity(0.4);
		//pPdfPainter->drawPixmap(5000, 100, 220 * 1.3, 110 * 1.3, pixmap);
		//pPdfPainter->drawPixmap(200, 100, 220 * 1.3, 110 * 1.3, pixmap);
		//pPdfPainter->setOpacity(1.0);


		//ͼֽ��С
		int pdfWidth = 2478;
		int pdfHeight = 3508;
		pPdfPainter->setFont(font[2]);
		pPdfPainter->setPen(QPen(Qt::darkGray, 1));




		pPdfPainter->drawLine(150, 270, 2328, 270);  //2478-150

		pPdfPainter->setFont(font[5]);
		pPdfPainter->setPen(QPen(Qt::black, 2));
		pPdfPainter->drawText(QRect(0, 0, 2478 / 3, 250), Qt::AlignHCenter | Qt::AlignBottom,
			"�������������޹�˾");



		//���Ʊ�����Ϣ
		int chartW1 = 2478 - 400;
		int chartH1 = 950;
		int chartL1 = 200;
		int chartT1 = 550;
		int chartRowH1 = 85;
		int charti1 = 0;
		pPdfPainter->setFont(font[1]);
		pPdfPainter->setPen(QPen(Qt::black, 3));

		pPdfPainter->drawText(QRect(150, 300, 2328, 200), Qt::AlignHCenter | Qt::AlignVCenter,
			QString("%1����").arg("λ��׼ȷ�Ⱥ��ظ��Բ���"));

		pPdfPainter->setFont(font[4]);
		pPdfPainter->setPen(QPen(Qt::black, 5));
		pPdfPainter->drawRect(chartL1, chartT1, chartW1, chartH1);
		pPdfPainter->setPen(QPen(Qt::black));
		pPdfPainter->setFont(font[2]);
		chartL1 += 15;
		pPdfPainter->drawText(QRect(chartL1 + 90, chartRowH1 * -1 + chartT1 + 12, pdfWidth / 4, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "���� ��9527");
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 340, chartRowH1 * -1 + chartT1 + 12, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "��� ��         ");

		pPdfPainter->setFont(font[6]);
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("            %1").arg("�ڿ�������ϵͳ"));

		pPdfPainter->setFont(font[2]);
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 260, chartRowH1 * charti1 + chartT1 + 10, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "�� �� Ա ��         ");
		charti1++;
		pPdfPainter->drawLine(pdfWidth / 2 + 500, chartRowH1 * charti1 + chartT1 + 5, chartW1 + 100, chartRowH1 * charti1 + chartT1 + 5);

		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("��Ʒ�ͺ� ��%1").arg(system_proc));
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 260, chartRowH1 * charti1 + chartT1 + 10, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "�������� ��         ");
		charti1++;
		pPdfPainter->drawLine(pdfWidth / 2 + 500, chartRowH1 * charti1 + chartT1 + 5, chartW1 + 100, chartRowH1 * charti1 + chartT1 + 5);
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�� �� �� ��%1").arg(system_serial));
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 260, chartRowH1 * charti1 + chartT1 + 10, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "�� �� Ա ��         ");
		charti1++;
		pPdfPainter->drawLine(pdfWidth / 2 + 500, chartRowH1 * charti1 + chartT1 + 5, chartW1 + 100, chartRowH1 * charti1 + chartT1 + 5);
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�������� ��%1").arg(system_batch));
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 260, chartRowH1 * charti1 + chartT1 + 10, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "������� ��         ");
		charti1++;
		pPdfPainter->drawLine(pdfWidth / 2 + 500, chartRowH1 * charti1 + chartT1 + 5, chartW1 + 100, chartRowH1 * charti1 + chartT1 + 5);
		//pPdfPainter->drawLine(chartL1 + 200, chartRowH1*charti1 + chartT1 + 10, chartW1 + 100, chartRowH1*charti1 + chartT1 + 10);


		pPdfPainter->setFont(font[6]);
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("            %1").arg("˫����ͷ��ǯ"));
		pPdfPainter->setFont(font[2]);
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("��Ʒ�ͺ� ��%1").arg(tool_proc));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�� �� �� ��%1").arg(tool_serial));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�������� ��%1").arg(tool_batch));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�豸���� ��%1").arg(QString::fromStdString("λ��׼ȷ�Ⱥ��ظ��Բ��Թ�װ")));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�豸��� ��%1").arg(QString::fromStdString(" ")));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�����׼ ��%1").arg(QString::fromStdString("������ SR-ENS-600 ����Ҫ��")));
		//pPdfPainter->drawLine(pdfWidth / 2 - 160, chartRowH1*charti1 + chartT1 + 10, chartW1 - 100, chartRowH1*charti1 + chartT1 + 10);
		pPdfPainter->setFont(font[2]);


		//��񡪡�λ��׼ȷ�Ⱥ��ظ��Բ���
		int chartW2 = 2478 - 400;
		int chartH2 = 700;
		int chartL2 = 200;
		int chartT2 = 1600;
		int chartRowH2 = 70;
		int charti2 = 0;
		pPdfPainter->setPen(QPen(Qt::black, 5));
		pPdfPainter->drawRect(chartL2, chartT2, chartW2, chartH2);

		//pPdfPainter->setFont(font[1]);
		//pPdfPainter->drawText(QRect(chartL2, chartT2 - 159, chartW2, 150), Qt::AlignHCenter | Qt::AlignVCenter,
		//	"λ��׼ȷ�Ⱥ��ظ��Բ���");

		pPdfPainter->setPen(QPen(Qt::black, 3));

		pPdfPainter->setFont(font[3]);
		pPdfPainter->drawText(QRect(chartL2, chartT2, 461, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			" ");
		pPdfPainter->drawText(QRect(chartL2 + 461, chartT2, 1155, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"ƽ���������");
		pPdfPainter->drawText(QRect(chartL2 + 1616, chartT2, 231, chartRowH2 * 2), Qt::AlignHCenter | Qt::AlignVCenter,
			"ָ��Ҫ��");
		pPdfPainter->drawText(QRect(chartL2 + 1847, chartT2, 231, chartRowH2 * 2), Qt::AlignHCenter | Qt::AlignVCenter,
			"�Ƿ�ͨ��");
		//����
		pPdfPainter->drawLine(chartL2 + 461, chartT2, chartL2 + 461, chartT2 + chartH2);
		pPdfPainter->drawLine(chartL2 + 1616, chartT2, chartL2 + 1616, chartT2 + chartH2);
		pPdfPainter->drawLine(chartL2 + 1847, chartT2, chartL2 + 1847, chartT2 + chartH2);

		pPdfPainter->drawLine(chartL2 + 692, chartT2 + chartRowH2, chartL2 + 692, chartT2 + chartH2);
		pPdfPainter->drawLine(chartL2 + 923, chartT2 + chartRowH2, chartL2 + 923, chartT2 + chartH2);
		pPdfPainter->drawLine(chartL2 + 1154, chartT2 + chartRowH2, chartL2 + 1154, chartT2 + chartH2);
		pPdfPainter->drawLine(chartL2 + 1385, chartT2 + chartRowH2, chartL2 + 1385, chartT2 + chartH2);
		pPdfPainter->drawLine(chartL2 + 1616, chartT2 + chartRowH2, chartL2 + 1616, chartT2 + chartH2);

		pPdfPainter->drawLine(chartL2 + 350, chartT2 + chartRowH2 * 4, chartL2 + 350, chartT2 + chartH2);



		//����
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2, chartL2 + chartW2 - 462, chartT2 + chartRowH2);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 2, chartL2 + chartW2, chartT2 + chartRowH2 * 2);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 3, chartL2 + chartW2, chartT2 + chartRowH2 * 3);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 4, chartL2 + chartW2, chartT2 + chartRowH2 * 4);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 7, chartL2 + chartW2, chartT2 + chartRowH2 * 7);

		pPdfPainter->drawLine(chartL2 + 350, chartT2 + chartRowH2 * 5, chartL2 + chartW2 - 231, chartT2 + chartRowH2 * 5);
		pPdfPainter->drawLine(chartL2 + 350, chartT2 + chartRowH2 * 6, chartL2 + chartW2 - 231, chartT2 + chartRowH2 * 6);
		pPdfPainter->drawLine(chartL2 + 350, chartT2 + chartRowH2 * 8, chartL2 + chartW2 - 231, chartT2 + chartRowH2 * 8);
		pPdfPainter->drawLine(chartL2 + 350, chartT2 + chartRowH2 * 9, chartL2 + chartW2 - 231, chartT2 + chartRowH2 * 9);
		//pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 6, chartL2 + chartW2, chartT2 + chartRowH2 * 6);

		pPdfPainter->setFont(font[3]);


		//��log
		qreal array[8][5] = { 0 };



		QTextStream in(&file);

		int line_number = 0;
		while (!in.atEnd()) {
			QString line = in.readLine();
			QStringList str_list = line.split(QRegExp("\\s{1,}"));
			for (int i = 0; i < str_list.length(); ++i) {
				array[line_number][i] = str_list[i].toFloat();
			}
			++line_number;
		}
		file.close();


		//�������
		pPdfPainter->drawText(QRect(chartL2, chartT2 + chartRowH2, 461, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"λ�˵�");
		pPdfPainter->drawText(QRect(chartL2 + 461, chartT2 + chartRowH2, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"1");
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 1, chartT2 + chartRowH2, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"2");
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 2, chartT2 + chartRowH2, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"3");
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 3, chartT2 + chartRowH2, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"4");
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 4, chartT2 + chartRowH2, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"5");

		pPdfPainter->drawText(QRect(chartL2, chartT2 + chartRowH2 * 2, 461, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg("λ��׼ȷ��(mm)"));
		pPdfPainter->drawText(QRect(chartL2 + 461, chartT2 + chartRowH2 * 2, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[0][0], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 1, chartT2 + chartRowH2 * 2, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[0][1], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 2, chartT2 + chartRowH2 * 2, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[0][2], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 3, chartT2 + chartRowH2 * 2, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[0][3], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 4, chartT2 + chartRowH2 * 2, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[0][4], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 5, chartT2 + chartRowH2 * 2, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"�� 2 ");
		if ((array[0][0] <= 2) && (array[0][1] <= 2) && (array[0][2] <= 2) && (array[0][3] <= 2) && (array[0][4] <= 2))
		{
			pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 6, chartT2 + chartRowH2 * 2, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"ͨ��");
		}

		else
		{
			pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 6, chartT2 + chartRowH2 * 2, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"δͨ��");
		}


		pPdfPainter->drawText(QRect(chartL2, chartT2 + chartRowH2 * 3, 461, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg("λ���ظ���(mm)"));
		pPdfPainter->drawText(QRect(chartL2 + 461, chartT2 + chartRowH2 * 3, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[1][0], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 1, chartT2 + chartRowH2 * 3, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[1][1], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 2, chartT2 + chartRowH2 * 3, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[1][2], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 3, chartT2 + chartRowH2 * 3, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[1][3], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 4, chartT2 + chartRowH2 * 3, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[1][4], 'f', 2)));

		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 5, chartT2 + chartRowH2 * 3, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"�� 2 ");

		////���ֵ
		//QVector <double> data1{ array[1][0], array[1][1], array[1][2], array[1][3], array[1][4] };
		//auto max1 = std::max_element(std::begin(data1), std::end(data1));
		//double biggest = *max1;
		//pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 4, chartT2 + chartRowH2 * 3, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
		//	QString(" %1 ").arg(QString::number(biggest)));

		if ((array[1][0] <= 2) && (array[1][1] <= 2) && (array[1][2] <= 2) && (array[1][3] <= 2) && (array[1][4] <= 2))
		{
			pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 6, chartT2 + chartRowH2 * 3, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"ͨ��");
		}

		else
		{
			pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 6, chartT2 + chartRowH2 * 3, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"δͨ��");
		}

		pPdfPainter->drawText(QRect(chartL2, chartT2 + chartRowH2 * 4, 350, chartRowH2 * 3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg("��̬׼ȷ��(��)"));
		pPdfPainter->drawText(QRect(chartL2 + 350, chartT2 + chartRowH2 * 4, 111, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg("X"));
		pPdfPainter->drawText(QRect(chartL2 + 461, chartT2 + chartRowH2 * 4, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[2][0], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 1, chartT2 + chartRowH2 * 4, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[2][1], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 2, chartT2 + chartRowH2 * 4, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[2][2], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 3, chartT2 + chartRowH2 * 4, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[2][3], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 4, chartT2 + chartRowH2 * 4, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[2][4], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 5, chartT2 + chartRowH2 * 4, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"�� 2 ");


		pPdfPainter->drawText(QRect(chartL2 + 350, chartT2 + chartRowH2 * 5, 111, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg("Y"));
		pPdfPainter->drawText(QRect(chartL2 + 461, chartT2 + chartRowH2 * 5, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[3][0], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 1, chartT2 + chartRowH2 * 5, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[3][1], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 2, chartT2 + chartRowH2 * 5, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[3][2], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 3, chartT2 + chartRowH2 * 5, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[3][3], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 4, chartT2 + chartRowH2 * 5, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[3][4], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 5, chartT2 + chartRowH2 * 5, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"�� 2 ");


		pPdfPainter->drawText(QRect(chartL2 + 350, chartT2 + chartRowH2 * 6, 111, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg("Z"));
		pPdfPainter->drawText(QRect(chartL2 + 461, chartT2 + chartRowH2 * 6, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[4][0], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 1, chartT2 + chartRowH2 * 6, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[4][1], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 2, chartT2 + chartRowH2 * 6, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[4][2], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 3, chartT2 + chartRowH2 * 6, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[4][3], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 4, chartT2 + chartRowH2 * 6, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[4][4], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 5, chartT2 + chartRowH2 * 6, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"�� 2 ");


		if ((array[2][0] <= 2) && (array[2][1] <= 2) && (array[2][2] <= 2) && (array[2][3] <= 2) && (array[2][4] <= 2) &&
			(array[3][0] <= 2) && (array[3][1] <= 2) && (array[3][2] <= 2) && (array[3][3] <= 2) && (array[3][4] <= 2) &&
			(array[4][0] <= 2) && (array[4][1] <= 2) && (array[4][2] <= 2) && (array[4][3] <= 2) && (array[4][4] <= 2)) {
			pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 6, chartT2 + chartRowH2 * 4, 231, chartRowH2 * 3), Qt::AlignHCenter | Qt::AlignVCenter,
				"ͨ��");
		}
		else
		{
			pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 6, chartT2 + chartRowH2 * 4, 231, chartRowH2 * 3), Qt::AlignHCenter | Qt::AlignVCenter,
				"δͨ��");
		}


		pPdfPainter->drawText(QRect(chartL2, chartT2 + chartRowH2 * 7, 350, chartRowH2 * 3), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg("��̬�ظ���(��)"));
		pPdfPainter->drawText(QRect(chartL2 + 350, chartT2 + chartRowH2 * 7, 111, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg("X"));
		pPdfPainter->drawText(QRect(chartL2 + 461, chartT2 + chartRowH2 * 7, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[5][0], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 1, chartT2 + chartRowH2 * 7, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[5][1], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 2, chartT2 + chartRowH2 * 7, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[5][2], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 3, chartT2 + chartRowH2 * 7, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[5][3], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 4, chartT2 + chartRowH2 * 7, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[5][4], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 5, chartT2 + chartRowH2 * 7, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"�� 2 ");


		pPdfPainter->drawText(QRect(chartL2 + 350, chartT2 + chartRowH2 * 8, 111, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg("Y"));
		pPdfPainter->drawText(QRect(chartL2 + 461, chartT2 + chartRowH2 * 8, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[6][0], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 1, chartT2 + chartRowH2 * 8, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[6][1], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 2, chartT2 + chartRowH2 * 8, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[6][2], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 3, chartT2 + chartRowH2 * 8, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[6][3], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 4, chartT2 + chartRowH2 * 8, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[6][4], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 5, chartT2 + chartRowH2 * 8, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"�� 2 ");


		pPdfPainter->drawText(QRect(chartL2 + 350, chartT2 + chartRowH2 * 9, 111, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg("Z"));
		pPdfPainter->drawText(QRect(chartL2 + 461, chartT2 + chartRowH2 * 9, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[7][0], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 1, chartT2 + chartRowH2 * 9, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[7][1], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 2, chartT2 + chartRowH2 * 9, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[7][2], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 3, chartT2 + chartRowH2 * 9, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[7][3], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 4, chartT2 + chartRowH2 * 9, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[7][4], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 5, chartT2 + chartRowH2 * 9, 231, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"�� 2 ");


		if ((array[5][0] <= 2) && (array[5][1] <= 2) && (array[5][2] <= 2) && (array[5][3] <= 2) && (array[5][4] <= 2) &&
			(array[6][0] <= 2) && (array[6][1] <= 2) && (array[6][2] <= 2) && (array[6][3] <= 2) && (array[6][4] <= 2) &&
			(array[7][0] <= 2) && (array[7][1] <= 2) && (array[7][2] <= 2) && (array[7][3] <= 2) && (array[7][4] <= 2)) {
			pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 6, chartT2 + chartRowH2 * 7, 231, chartRowH2 * 3), Qt::AlignHCenter | Qt::AlignVCenter,
				"ͨ��");
		}
		else
		{
			pPdfPainter->drawText(QRect(chartL2 + 461 + 231 * 6, chartT2 + chartRowH2 * 7, 231, chartRowH2 * 3), Qt::AlignHCenter | Qt::AlignVCenter,
				"δͨ��");
		}

		//ҳ��
		pPdfPainter->setFont(font[3]);
		pPdfPainter->setPen(QPen(Qt::darkGray, 1));
		pPdfPainter->drawLine(150, 3310, 2328, 3310);  //2478-150

		pPdfPainter->drawText(QRect(1100, 3200, 300, 300), Qt::AlignHCenter | Qt::AlignVCenter,
			"1");
		pPdfPainter->drawText(QRect(1820, 3200, 570, 300), Qt::AlignHCenter | Qt::AlignVCenter,
			"������Ϣ �ϸ���");
		pPdfPainter->drawText(QRect(50, 3200, 700, 300), Qt::AlignHCenter | Qt::AlignVCenter,
			QString("%1").arg("SR-ENS-600"));
		pPdfPainter->setFont(font[2]);




		delete pPdfPainter;
		delete pPdfWriter;
		pdfFile.close();
		//������ǰ·������Ϊԭ����·��


		tempDir.setCurrent(currentDir);
		delete m_pProgressBar;
		QMessageBox::warning(this, "��ʾ", "�������");
		setTipWindow("λ��׼ȷ�Ⱥ��ظ��Բ��Ա������ɳɹ�");
	}
}

void GenRep::CreatePdfMaxWorkspaceSlot()
{
	setTipWindow("�����������ռ���Ա�����������...");
	QString str = file_path + "work_space.log";
	QFile file(str);

	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) 
	{
		setTipWindow("�����������ռ���Ա�������ʧ��");
		return;
	}
	//�������ǰ·��
	QString currentDir = QDir::currentPath();
	//����pdf�����·��
	QString file_path = QFileDialog::getExistingDirectory(this, "��ѡ�񱣴�·��", "C:\\Users\\Administrator\\Desktop");
	if (file_path.isEmpty())
	{
		setTipWindow("�����������ռ���Ա�������ʧ��");
		QMessageBox::warning(this, "��ʾ", "·������Ϊ��");
		return;
		
	}

	//�������ִ��·�����õ�filePath��
	QDir tempDir;
	tempDir.setCurrent(file_path);

	//������
	QProgressBar* m_pProgressBar = new QProgressBar(this);
	m_pProgressBar->setOrientation(Qt::Horizontal);  // ˮƽ����
	m_pProgressBar->setMinimum(1);  // ��Сֵ
	m_pProgressBar->setMaximum(1);  // ���ֵ str.size()
	m_pProgressBar->setMaximumHeight(15);
	m_pProgressBar->setMinimumWidth(60);
	m_pProgressBar->setVisible(true);


	m_pProgressBar->setValue(0);
	QCoreApplication::processEvents();

	QDate date = QDate::currentDate();

	//QString serialNum = QString::number(Production.at(ui.toolComboBox->currentIndex()).second);
	//QString year = str.at(3).left(4);

	QString fileName = QString::fromStdString("�����������ռ����") + ".pdf";
	QFile pdfFile(fileName);
	//�ж��ļ��Ƿ����
	if (QFile::exists(fileName))
	{
		QMessageBox::StandardButton reply;
		reply = QMessageBox::question(this, "��ʾ", "�ļ��Ѿ�����,����ǽ�����ԭ�ļ�", QMessageBox::Yes | QMessageBox::No);
		if (reply == QMessageBox::Yes)
		{
			QFile::remove(fileName);
		}
		else
		{
			setTipWindow("�����������ռ���Ա�������ʧ��");
			return;
		}
	}
	if (1)
	{
		// ��Ҫд���pdf�ļ�
		pdfFile.open(QIODevice::WriteOnly);

		QPdfWriter* pPdfWriter = new QPdfWriter(&pdfFile);  // ����pdfд����
		pPdfWriter->setPageSize(QPagedPaintDevice::A4);     // ����ֽ��ΪA4
		pPdfWriter->setResolution(300);                     // ����ֽ�ŵķֱ���Ϊ300,���������Ϊ3508X2479

		int iMargin = 0;                   // ҳ�߾�
		pPdfWriter->setPageMargins(QMarginsF(iMargin, iMargin, iMargin, iMargin));

		QPainter* pPdfPainter = new QPainter(pPdfWriter);   // qt���ƹ���

		// ����,����
		QTextOption option(Qt::AlignHCenter | Qt::AlignVCenter);
		option.setWrapMode(QTextOption::WordWrap);

		//����
		//QFont font;
		//font.setFamily("simhei.ttf");
		QFont font[8] = { QFont("����", 26, 60), QFont("����", 26, 61), QFont("����", 26, QFont::Normal), QFont("times new roman", 26, QFont::Normal), QFont("����", 26, QFont::Normal), QFont("������κ", 26, 61), QFont("����", 26, QFont::Normal), QFont("times new roman", 26, QFont::Normal) };
		font[0].setPixelSize(86);
		font[1].setPixelSize(55);
		font[2].setPixelSize(40);
		font[3].setPixelSize(40);
		font[4].setPixelSize(50);
		font[5].setPixelSize(50);
		font[6].setPixelSize(40);
		font[7].setPixelSize(50);
		//Painter PDF
		qDebug() << pPdfPainter->viewport();
		int nPDFWidth = pPdfPainter->viewport().width();
		int nPDFHeight = pPdfPainter->viewport().height();

		//pPdfPainter->setPen(QPen(Qt::black, 5));
		//pPdfPainter->drawLine(0, 0, 2478, 0);
		//pPdfPainter->size
		//����ͼƬˮƽ�߾�Ϊ150
		//pPdfPainter->setOpacity(0.4);
		//pPdfPainter->drawPixmap(5000, 100, 220 * 1.3, 110 * 1.3, pixmap);
		//pPdfPainter->drawPixmap(200, 100, 220 * 1.3, 110 * 1.3, pixmap);
		//pPdfPainter->setOpacity(1.0);


		//ͼֽ��С
		int pdfWidth = 2478;
		int pdfHeight = 3508;
		pPdfPainter->setFont(font[2]);
		pPdfPainter->setPen(QPen(Qt::darkGray, 1));




		pPdfPainter->drawLine(150, 270, 2328, 270);  //2478-150

		pPdfPainter->setFont(font[5]);
		pPdfPainter->setPen(QPen(Qt::black, 2));
		pPdfPainter->drawText(QRect(0, 0, 2478 / 3, 250), Qt::AlignHCenter | Qt::AlignBottom,
			"�������������޹�˾");


		//���Ʊ�����Ϣ
		int chartW1 = 2478 - 400;
		int chartH1 = 950;
		int chartL1 = 200;
		int chartT1 = 550;
		int chartRowH1 = 85;
		int charti1 = 0;
		pPdfPainter->setFont(font[1]);
		pPdfPainter->setPen(QPen(Qt::black, 3));

		pPdfPainter->drawText(QRect(150, 300, 2328, 200), Qt::AlignHCenter | Qt::AlignVCenter,
			QString("%1����").arg("�����������ռ����"));

		pPdfPainter->setFont(font[4]);
		pPdfPainter->setPen(QPen(Qt::black, 5));
		pPdfPainter->drawRect(chartL1, chartT1, chartW1, chartH1);
		pPdfPainter->setPen(QPen(Qt::black));
		pPdfPainter->setFont(font[2]);
		chartL1 += 15;
		pPdfPainter->drawText(QRect(chartL1 + 90, chartRowH1 * -1 + chartT1 + 12, pdfWidth / 4, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "���� ��9527");
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 340, chartRowH1 * -1 + chartT1 + 12, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "��� ��         ");

		pPdfPainter->setFont(font[6]);
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("            %1").arg("�ڿ�������ϵͳ"));

		pPdfPainter->setFont(font[2]);
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 260, chartRowH1 * charti1 + chartT1 + 10, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "�� �� Ա ��         ");
		charti1++;
		pPdfPainter->drawLine(pdfWidth / 2 + 500, chartRowH1 * charti1 + chartT1 + 5, chartW1 + 100, chartRowH1 * charti1 + chartT1 + 5);

		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("��Ʒ�ͺ� ��%1").arg(system_proc));
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 260, chartRowH1 * charti1 + chartT1 + 10, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "�������� ��         ");
		charti1++;
		pPdfPainter->drawLine(pdfWidth / 2 + 500, chartRowH1 * charti1 + chartT1 + 5, chartW1 + 100, chartRowH1 * charti1 + chartT1 + 5);
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�� �� �� ��%1").arg(system_serial));
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 260, chartRowH1 * charti1 + chartT1 + 10, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "�� �� Ա ��         ");
		charti1++;
		pPdfPainter->drawLine(pdfWidth / 2 + 500, chartRowH1 * charti1 + chartT1 + 5, chartW1 + 100, chartRowH1 * charti1 + chartT1 + 5);
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�������� ��%1").arg(system_batch));
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 260, chartRowH1 * charti1 + chartT1 + 10, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "������� ��         ");
		charti1++;
		pPdfPainter->drawLine(pdfWidth / 2 + 500, chartRowH1 * charti1 + chartT1 + 5, chartW1 + 100, chartRowH1 * charti1 + chartT1 + 5);
		//pPdfPainter->drawLine(chartL1 + 200, chartRowH1*charti1 + chartT1 + 10, chartW1 + 100, chartRowH1*charti1 + chartT1 + 10);


		pPdfPainter->setFont(font[6]);
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("            %1").arg("˫����ͷ��ǯ"));
		pPdfPainter->setFont(font[2]);
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("��Ʒ�ͺ� ��%1").arg(tool_proc));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�� �� �� ��%1").arg(tool_serial));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�������� ��%1").arg(tool_batch));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�豸���� ��%1").arg(QString::fromStdString("�����������ռ���Թ�װ")));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�豸��� ��%1").arg(QString::fromStdString(" ")));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�����׼ ��%1").arg(QString::fromStdString("������ SR-ENS-600 ����Ҫ��")));
		//pPdfPainter->drawLine(pdfWidth / 2 - 160, chartRowH1*charti1 + chartT1 + 10, chartW1 - 100, chartRowH1*charti1 + chartT1 + 10);
		pPdfPainter->setFont(font[2]);





		//��񡪡�������ռ����
		int chartW2 = 2478 - 400;
		int chartH2 = 490;
		int chartL2 = 200;
		int chartT2 = 1600;
		int chartRowH2 = 70;
		int charti2 = 0;
		pPdfPainter->setPen(QPen(Qt::black, 5));
		pPdfPainter->drawRect(chartL2, chartT2, chartW2, chartH2);

		//pPdfPainter->setFont(font[1]);
		//pPdfPainter->drawText(QRect(chartL2, chartT2 - 159, chartW2, 150), Qt::AlignHCenter | Qt::AlignVCenter,
		//	"������ռ����");

		pPdfPainter->setPen(QPen(Qt::black, 3));

		pPdfPainter->setFont(font[3]);
		pPdfPainter->drawText(QRect(chartL2, chartT2, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			" ");
		pPdfPainter->drawText(QRect(chartL2 + 500, chartT2, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"ƽ���������");
		pPdfPainter->drawText(QRect(chartL2 + 1000, chartT2, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"ָ��Ҫ��");
		pPdfPainter->drawText(QRect(chartL2 + 1500, chartT2, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"�Ƿ�ͨ��");
		//����
		pPdfPainter->drawLine(chartL2 + 500, chartT2, chartL2 + 500, chartT2 + chartH2);
		pPdfPainter->drawLine(chartL2 + 1000, chartT2, chartL2 + 1000, chartT2 + chartH2);
		pPdfPainter->drawLine(chartL2 + 1500, chartT2, chartL2 + 1500, chartT2 + chartH2);
		//����
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2, chartL2 + chartW2, chartT2 + chartRowH2);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 2, chartL2 + chartW2, chartT2 + chartRowH2 * 2);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 3, chartL2 + chartW2, chartT2 + chartRowH2 * 3);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 4, chartL2 + chartW2, chartT2 + chartRowH2 * 4);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 5, chartL2 + chartW2, chartT2 + chartRowH2 * 5);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 6, chartL2 + chartW2, chartT2 + chartRowH2 * 6);

		pPdfPainter->setFont(font[3]);


		//��log
		qreal array[1][6] = { 0 };



		QTextStream in(&file);

		int line_number = 0;
		while (!in.atEnd()) {
			QString line = in.readLine();
			QStringList str_list = line.split(QRegExp("\\s{1,}"));
			for (int i = 0; i < str_list.length(); ++i) {
				array[line_number][i] = str_list[i].toFloat();
			}
			++line_number;
		}
		file.close();

		//�������
		pPdfPainter->drawText(QRect(chartL2, chartT2 + chartRowH2, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"X��������");
		pPdfPainter->drawText(QRect(chartL2 + 500, chartT2 + chartRowH2, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 mm").arg(QString::number(array[0][0], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 1000, chartT2 + chartRowH2, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"-155��5 mm");
		if (array[0][0] > -150)
			pPdfPainter->drawText(QRect(chartL2 + 1500, chartT2 + chartRowH2, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"δͨ��");
		else if (array[0][0] < -160)
			pPdfPainter->drawText(QRect(chartL2 + 1500, chartT2 + chartRowH2, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"δͨ��");
		else
			pPdfPainter->drawText(QRect(chartL2 + 1500, chartT2 + chartRowH2, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"ͨ��");


		pPdfPainter->drawText(QRect(chartL2, chartT2 + chartRowH2 * 2, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"X��������");
		pPdfPainter->drawText(QRect(chartL2 + 500, chartT2 + chartRowH2 * 2, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 mm").arg(QString::number(array[0][1], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 1000, chartT2 + chartRowH2 * 2, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"155��5 mm");
		if (array[0][1] > 160)
			pPdfPainter->drawText(QRect(chartL2 + 1500, chartT2 + chartRowH2 * 2, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"δͨ��");
		else if (array[0][1] < 150)
			pPdfPainter->drawText(QRect(chartL2 + 1500, chartT2 + chartRowH2 * 2, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"δͨ��");
		else
			pPdfPainter->drawText(QRect(chartL2 + 1500, chartT2 + chartRowH2 * 2, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"ͨ��");


		pPdfPainter->drawText(QRect(chartL2, chartT2 + chartRowH2 * 3, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"Y��������");
		pPdfPainter->drawText(QRect(chartL2 + 500, chartT2 + chartRowH2 * 3, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 mm").arg(QString::number(array[0][2], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 1000, chartT2 + chartRowH2 * 3, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"-160��5 mm");
		if (array[0][2] > -155)
			pPdfPainter->drawText(QRect(chartL2 + 1500, chartT2 + chartRowH2 * 3, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"δͨ��");
		else if (array[0][2] < -165)
			pPdfPainter->drawText(QRect(chartL2 + 1500, chartT2 + chartRowH2 * 3, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"δͨ��");
		else
			pPdfPainter->drawText(QRect(chartL2 + 1500, chartT2 + chartRowH2 * 3, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"ͨ��");

		pPdfPainter->drawText(QRect(chartL2, chartT2 + chartRowH2 * 4, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"Y��������");
		pPdfPainter->drawText(QRect(chartL2 + 500, chartT2 + chartRowH2 * 4, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 mm").arg(QString::number(array[0][3], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 1000, chartT2 + chartRowH2 * 4, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"160��5 mm");
		if (array[0][3] > 165)
			pPdfPainter->drawText(QRect(chartL2 + 1500, chartT2 + chartRowH2 * 4, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"δͨ��");
		else if (array[0][3] < 155)
			pPdfPainter->drawText(QRect(chartL2 + 1500, chartT2 + chartRowH2 * 4, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"δͨ��");
		else
			pPdfPainter->drawText(QRect(chartL2 + 1500, chartT2 + chartRowH2 * 4, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"ͨ��");


		pPdfPainter->drawText(QRect(chartL2, chartT2 + chartRowH2 * 5, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"Z��������");
		pPdfPainter->drawText(QRect(chartL2 + 500, chartT2 + chartRowH2 * 5, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 mm").arg(QString::number(array[0][4], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 1000, chartT2 + chartRowH2 * 5, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"-45��5 mm");
		if (array[0][4] > -40)
			pPdfPainter->drawText(QRect(chartL2 + 1500, chartT2 + chartRowH2 * 5, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"δͨ��");
		else if (array[0][4] < -50)
			pPdfPainter->drawText(QRect(chartL2 + 1500, chartT2 + chartRowH2 * 5, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"δͨ��");
		else
			pPdfPainter->drawText(QRect(chartL2 + 1500, chartT2 + chartRowH2 * 5, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"ͨ��");


		pPdfPainter->drawText(QRect(chartL2, chartT2 + chartRowH2 * 6, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"Z��������");
		pPdfPainter->drawText(QRect(chartL2 + 500, chartT2 + chartRowH2 * 6, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 mm").arg(QString::number(array[0][5], 'f', 2)));
		pPdfPainter->drawText(QRect(chartL2 + 1000, chartT2 + chartRowH2 * 6, 500, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"220��5 mm");
		if (array[0][5] > 225)
			pPdfPainter->drawText(QRect(chartL2 + 1500, chartT2 + chartRowH2 * 6, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"δͨ��");
		else if (array[0][5] < 215)
			pPdfPainter->drawText(QRect(chartL2 + 1500, chartT2 + chartRowH2 * 6, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"δͨ��");
		else
			pPdfPainter->drawText(QRect(chartL2 + 1500, chartT2 + chartRowH2 * 6, 578, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"ͨ��");





		//ҳ��
		pPdfPainter->setFont(font[3]);
		pPdfPainter->setPen(QPen(Qt::darkGray, 1));
		pPdfPainter->drawLine(150, 3310, 2328, 3310);  //2478-150

		pPdfPainter->drawText(QRect(1100, 3200, 300, 300), Qt::AlignHCenter | Qt::AlignVCenter,
			"1");
		pPdfPainter->drawText(QRect(1820, 3200, 570, 300), Qt::AlignHCenter | Qt::AlignVCenter,
			"������Ϣ �ϸ���");
		pPdfPainter->drawText(QRect(50, 3200, 700, 300), Qt::AlignHCenter | Qt::AlignVCenter,
			QString("%1").arg("SR-ENS-600"));
		pPdfPainter->setFont(font[2]);




		delete pPdfPainter;
		delete pPdfWriter;
		pdfFile.close();
		//������ǰ·������Ϊԭ����·��


		tempDir.setCurrent(currentDir);
		delete m_pProgressBar;
		QMessageBox::warning(this, "��ʾ", "�������");
		setTipWindow("�����������ռ���Ա������ɳɹ�");
	}
}

void GenRep::CreatePdfDelaySlot()
{
	
	setTipWindow("���ӿ����ӳ�ʱ����Ա�����������...");
	QString str = file_path + "result_DELAY.log";
	QFile file(str);

	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) 
	{
		setTipWindow("���ӿ����ӳ�ʱ����Ա�������ʧ�ܡ�");
		return;
	}
	//�������ǰ·��
	QString currentDir = QDir::currentPath();
	//����pdf�����·��
	QString file_path = QFileDialog::getExistingDirectory(this, "��ѡ�񱣴�·��", "C:\\Users\\Administrator\\Desktop");
	if (file_path.isEmpty())
	{
		QMessageBox::warning(this, "��ʾ", "·������Ϊ��");
		setTipWindow("���ӿ����ӳ�ʱ����Ա�������ʧ�ܡ�");
		return;
		
	}

	//�������ִ��·�����õ�filePath��
	QDir tempDir;
	tempDir.setCurrent(file_path);

	//������
	QProgressBar* m_pProgressBar = new QProgressBar(this);
	m_pProgressBar->setOrientation(Qt::Horizontal);  // ˮƽ����
	m_pProgressBar->setMinimum(1);  // ��Сֵ
	m_pProgressBar->setMaximum(1);  // ���ֵ str.size()
	m_pProgressBar->setMaximumHeight(15);
	m_pProgressBar->setMinimumWidth(60);
	m_pProgressBar->setVisible(true);


	m_pProgressBar->setValue(0);
	QCoreApplication::processEvents();

	QDate date = QDate::currentDate();

	//QString serialNum = QString::number(Production.at(ui.toolComboBox->currentIndex()).second);
	//QString year = str.at(3).left(4);

	QString fileName = QString::fromStdString("���ӿ����ӳ�ʱ�����") + ".pdf";
	QFile pdfFile(fileName);
	//�ж��ļ��Ƿ����
	if (QFile::exists(fileName))
	{
		QMessageBox::StandardButton reply;
		reply = QMessageBox::question(this, "��ʾ", "�ļ��Ѿ�����,����ǽ�����ԭ�ļ�", QMessageBox::Yes | QMessageBox::No);
		if (reply == QMessageBox::Yes)
		{
			QFile::remove(fileName);
		}
		else
		{
			setTipWindow("���ӿ����ӳ�ʱ����Ա�������ֹͣ��");
			return;
		}
	}
	if (1)
	{
		// ��Ҫд���pdf�ļ�
		pdfFile.open(QIODevice::WriteOnly);

		QPdfWriter* pPdfWriter = new QPdfWriter(&pdfFile);  // ����pdfд����
		pPdfWriter->setPageSize(QPagedPaintDevice::A4);     // ����ֽ��ΪA4
		pPdfWriter->setResolution(300);                     // ����ֽ�ŵķֱ���Ϊ300,���������Ϊ3508X2479

		int iMargin = 0;                   // ҳ�߾�
		pPdfWriter->setPageMargins(QMarginsF(iMargin, iMargin, iMargin, iMargin));

		QPainter* pPdfPainter = new QPainter(pPdfWriter);   // qt���ƹ���

		// ����,����
		QTextOption option(Qt::AlignHCenter | Qt::AlignVCenter);
		option.setWrapMode(QTextOption::WordWrap);

		//����
		//QFont font;
		//font.setFamily("simhei.ttf");
		QFont font[8] = { QFont("����", 26, 60), QFont("����", 26, 61), QFont("����", 26, QFont::Normal), QFont("times new roman", 26, QFont::Normal), QFont("����", 26, QFont::Normal), QFont("������κ", 26, 61), QFont("����", 26, QFont::Normal), QFont("times new roman", 26, QFont::Normal) };
		font[0].setPixelSize(86);
		font[1].setPixelSize(55);
		font[2].setPixelSize(40);
		font[3].setPixelSize(40);
		font[4].setPixelSize(50);
		font[5].setPixelSize(50);
		font[6].setPixelSize(40);
		font[7].setPixelSize(50);
		//Painter PDF
		qDebug() << pPdfPainter->viewport();
		int nPDFWidth = pPdfPainter->viewport().width();
		int nPDFHeight = pPdfPainter->viewport().height();

		//pPdfPainter->setPen(QPen(Qt::black, 5));
		//pPdfPainter->drawLine(0, 0, 2478, 0);
		//pPdfPainter->size
		//����ͼƬˮƽ�߾�Ϊ150
		//pPdfPainter->setOpacity(0.4);
		//pPdfPainter->drawPixmap(5000, 100, 220 * 1.3, 110 * 1.3, pixmap);
		//pPdfPainter->drawPixmap(200, 100, 220 * 1.3, 110 * 1.3, pixmap);
		//pPdfPainter->setOpacity(1.0);


		//ͼֽ��С
		int pdfWidth = 2478;
		int pdfHeight = 3508;
		pPdfPainter->setFont(font[2]);
		pPdfPainter->setPen(QPen(Qt::darkGray, 1));




		pPdfPainter->drawLine(150, 270, 2328, 270);  //2478-150

		pPdfPainter->setFont(font[5]);
		pPdfPainter->setPen(QPen(Qt::black, 2));
		pPdfPainter->drawText(QRect(0, 0, 2478 / 3, 250), Qt::AlignHCenter | Qt::AlignBottom,
			"�������������޹�˾");


		//���Ʊ�����Ϣ
		int chartW1 = 2478 - 400;
		int chartH1 = 950;
		int chartL1 = 200;
		int chartT1 = 550;
		int chartRowH1 = 85;
		int charti1 = 0;
		pPdfPainter->setFont(font[1]);
		pPdfPainter->setPen(QPen(Qt::black, 3));

		pPdfPainter->drawText(QRect(150, 300, 2328, 200), Qt::AlignHCenter | Qt::AlignVCenter,
			QString("%1����").arg("���ӿ����ӳ�ʱ�����"));

		pPdfPainter->setFont(font[4]);
		pPdfPainter->setPen(QPen(Qt::black, 5));
		pPdfPainter->drawRect(chartL1, chartT1, chartW1, chartH1);
		pPdfPainter->setPen(QPen(Qt::black));
		pPdfPainter->setFont(font[2]);
		chartL1 += 15;
		pPdfPainter->drawText(QRect(chartL1 + 90, chartRowH1 * -1 + chartT1 + 12, pdfWidth / 4, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "���� ��9527");
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 340, chartRowH1 * -1 + chartT1 + 12, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "��� ��         ");

		pPdfPainter->setFont(font[6]);
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("            %1").arg("�ڿ�������ϵͳ"));

		pPdfPainter->setFont(font[2]);
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 260, chartRowH1 * charti1 + chartT1 + 10, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "�� �� Ա ��         ");
		charti1++;
		pPdfPainter->drawLine(pdfWidth / 2 + 500, chartRowH1 * charti1 + chartT1 + 5, chartW1 + 100, chartRowH1 * charti1 + chartT1 + 5);

		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("��Ʒ�ͺ� ��%1").arg(system_proc));
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 260, chartRowH1 * charti1 + chartT1 + 10, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "�������� ��         ");
		charti1++;
		pPdfPainter->drawLine(pdfWidth / 2 + 500, chartRowH1 * charti1 + chartT1 + 5, chartW1 + 100, chartRowH1 * charti1 + chartT1 + 5);
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�� �� �� ��%1").arg(system_serial));
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 260, chartRowH1 * charti1 + chartT1 + 10, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "�� �� Ա ��         ");
		charti1++;
		pPdfPainter->drawLine(pdfWidth / 2 + 500, chartRowH1 * charti1 + chartT1 + 5, chartW1 + 100, chartRowH1 * charti1 + chartT1 + 5);
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�������� ��%1").arg(system_batch));
		pPdfPainter->drawText(QRect(pdfWidth / 2 + 260, chartRowH1 * charti1 + chartT1 + 10, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "������� ��         ");
		charti1++;
		pPdfPainter->drawLine(pdfWidth / 2 + 500, chartRowH1 * charti1 + chartT1 + 5, chartW1 + 100, chartRowH1 * charti1 + chartT1 + 5);
		//pPdfPainter->drawLine(chartL1 + 200, chartRowH1*charti1 + chartT1 + 10, chartW1 + 100, chartRowH1*charti1 + chartT1 + 10);


		pPdfPainter->setFont(font[6]);
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("            %1").arg("˫����ͷ��ǯ"));
		pPdfPainter->setFont(font[2]);
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("��Ʒ�ͺ� ��%1").arg(tool_proc));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�� �� �� ��%1").arg(tool_serial));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�������� ��%1").arg(tool_batch));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�豸���� ��%1").arg(QString::fromStdString("���ӿ����ӳ�ʱ����Թ�װ")));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�豸��� ��%1").arg(QString::fromStdString(" ")));
		charti1++;
		pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("�����׼ ��%1").arg(QString::fromStdString("������ SR-ENS-600 ����Ҫ��")));
		//pPdfPainter->drawLine(pdfWidth / 2 - 160, chartRowH1*charti1 + chartT1 + 10, chartW1 - 100, chartRowH1*charti1 + chartT1 + 10);
		pPdfPainter->setFont(font[2]);


		//��log
		qreal array[6][6] = { 0 };

		

		QTextStream in(&file);

		int line_number = 0;
		while (!in.atEnd()) {
			QString line = in.readLine();
			QStringList str_list = line.split(QRegExp(","));
			for (int i = 0; i < str_list.length(); ++i) {
				array[line_number][i] = str_list[i].toFloat();
			}
			++line_number;
		}
		file.close();


		//��񡪡����ӿ�����ʱ
		int chartW2 = 2478 - 400;
		int chartH2 = 700;
		int chartL2 = 200;
		int chartT2 = 1600;
		int chartRowH2 = 70;
		int charti2 = 0;
		pPdfPainter->setPen(QPen(Qt::black, 5));
		pPdfPainter->drawRect(chartL2, chartT2, chartW2, chartH2);

		//pPdfPainter->setFont(font[1]);
		//pPdfPainter->drawText(QRect(chartL2, chartT2 - 159, chartW2, 150), Qt::AlignHCenter | Qt::AlignVCenter,
		//	"���ӿ�����ʱ");

		pPdfPainter->setPen(QPen(Qt::black, 3));

		pPdfPainter->setFont(font[3]);



		pPdfPainter->drawText(QRect(chartL2, chartRowH2 * charti2 + chartT2, 338, chartRowH2 * 2), Qt::AlignHCenter | Qt::AlignVCenter,
			" ������ʱ ");
		pPdfPainter->drawText(QRect(chartL2 + 338, chartRowH2 * charti2 + chartT2, 290 * 2, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			" X (ms) ");
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 2, chartRowH2 * charti2 + chartT2, 290 * 2, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			" Y (ms) ");
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 4, chartRowH2 * charti2 + chartT2, 290 * 2, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			" Z (ms) ");



		charti2++;
		pPdfPainter->drawText(QRect(chartL2 + 338, chartRowH2 * charti2 + chartT2, 338, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"������ʱ");
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 1, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"������ʱ");
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 2, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"������ʱ");
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 3, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"������ʱ");
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 4, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"������ʱ");
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 5, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"������ʱ");

		charti2++;
		pPdfPainter->drawText(QRect(chartL2, chartRowH2 * charti2 + chartT2, 338, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"1");
		pPdfPainter->drawText(QRect(chartL2 + 338, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[0][0])));
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 1, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[0][1])));
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 2, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[0][2])));
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 3, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[0][3])));
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 4, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[0][4])));
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 5, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[0][5])));

		charti2++;
		pPdfPainter->drawText(QRect(chartL2, chartRowH2 * charti2 + chartT2, 338, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"2");
		pPdfPainter->drawText(QRect(chartL2 + 338, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[1][0])));
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 1, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[1][1])));
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 2, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[1][2])));
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 3, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[1][3])));
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 4, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[1][4])));
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 5, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[1][5])));

		charti2++;
		pPdfPainter->drawText(QRect(chartL2, chartRowH2 * charti2 + chartT2, 338, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"3");
		pPdfPainter->drawText(QRect(chartL2 + 338, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[2][0])));
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 1, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[2][1])));
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 2, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[2][2])));
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 3, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[2][3])));
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 4, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[2][4])));
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 5, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[2][5])));

		charti2++;
		pPdfPainter->drawText(QRect(chartL2, chartRowH2 * charti2 + chartT2, 338, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"ƽ��ֵ");
		pPdfPainter->drawText(QRect(chartL2 + 338, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[3][0])));
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 1, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[3][1])));
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 2, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[3][2])));
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 3, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[3][3])));
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 4, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[3][4])));
		pPdfPainter->drawText(QRect(chartL2 + 338 + 290 * 5, chartRowH2 * charti2 + chartT2, 290, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[3][5])));


		////���ֵ
		//QVector <double> data0{ array[0][0], array[1][0], array[2][0], array[0][2], array[1][2], array[2][2], array[0][4], array[1][4], array[2][4] };
		//auto max0 = std::max_element(std::begin(data0), std::end(data0));
		//double biggest0 = *max0;

		//QVector <double> data1{ array[0][1], array[1][1], array[2][1], array[0][3], array[1][3], array[2][3], array[0][5], array[1][5], array[2][5] };
		//auto max1 = std::max_element(std::begin(data1), std::end(data1));
		//double biggest1 = *max1;

		charti2++;
		pPdfPainter->drawText(QRect(chartL2, chartRowH2 * charti2 + chartT2, 338, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"������ʱ���ֵ");
		pPdfPainter->drawText(QRect(chartL2 + 338, chartRowH2 * charti2 + chartT2, 290 * 6, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[4][0])));

		charti2++;
		pPdfPainter->drawText(QRect(chartL2, chartRowH2 * charti2 + chartT2, 338, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"������ʱ���ֵ");
		pPdfPainter->drawText(QRect(chartL2 + 338, chartRowH2 * charti2 + chartT2, 290 * 6, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			QString(" %1 ").arg(QString::number(array[5][0])));

		charti2++;
		pPdfPainter->drawText(QRect(chartL2, chartRowH2 * charti2 + chartT2, 338, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"��׼ֵ");
		pPdfPainter->drawText(QRect(chartL2 + 338, chartRowH2 * charti2 + chartT2, 290 * 6, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"�� 200");

		charti2++;
		pPdfPainter->drawText(QRect(chartL2, chartRowH2 * charti2 + chartT2, 338, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
			"����");

		if ((array[4][0] <= 200) && (array[5][0] <= 200)) {
			pPdfPainter->drawText(QRect(chartL2 + 338, chartRowH2 * charti2 + chartT2, 290 * 6, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				QString(" %1 ").arg("�ϸ�"));
		}
		else
		{
			pPdfPainter->drawText(QRect(chartL2 + 338, chartRowH2 * charti2 + chartT2, 290 * 6, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				QString(" %1 ").arg("���ϸ�"));
		}





		//����
		pPdfPainter->drawLine(chartL2 + 338 + 290 * 0, chartT2, chartL2 + 338, chartT2 + chartH2);
		pPdfPainter->drawLine(chartL2 + 338 + 290 * 1, chartT2 + chartRowH2, chartL2 + 338 + 290 * 1, chartT2 + chartRowH2 * 6);
		pPdfPainter->drawLine(chartL2 + 338 + 290 * 3, chartT2 + chartRowH2, chartL2 + 338 + 290 * 3, chartT2 + chartRowH2 * 6);
		pPdfPainter->drawLine(chartL2 + 338 + 290 * 5, chartT2 + chartRowH2, chartL2 + 338 + 290 * 5, chartT2 + chartRowH2 * 6);
		pPdfPainter->drawLine(chartL2 + 338 + 290 * 2, chartT2, chartL2 + 338 + 290 * 2, chartT2 + chartRowH2 * 6);
		pPdfPainter->drawLine(chartL2 + 338 + 290 * 4, chartT2, chartL2 + 338 + 290 * 4, chartT2 + chartRowH2 * 6);


		//����
		pPdfPainter->drawLine(chartL2 + 338, chartT2 + chartRowH2, chartL2 + chartW2, chartT2 + chartRowH2);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 2, chartL2 + chartW2, chartT2 + chartRowH2 * 2);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 3, chartL2 + chartW2, chartT2 + chartRowH2 * 3);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 4, chartL2 + chartW2, chartT2 + chartRowH2 * 4);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 5, chartL2 + chartW2, chartT2 + chartRowH2 * 5);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 6, chartL2 + chartW2, chartT2 + chartRowH2 * 6);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 7, chartL2 + chartW2, chartT2 + chartRowH2 * 7);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 8, chartL2 + chartW2, chartT2 + chartRowH2 * 8);
		pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 9, chartL2 + chartW2, chartT2 + chartRowH2 * 9);






		//ҳ��
		pPdfPainter->setFont(font[3]);
		pPdfPainter->setPen(QPen(Qt::darkGray, 1));
		pPdfPainter->drawLine(150, 3310, 2328, 3310);  //2478-150

		pPdfPainter->drawText(QRect(1100, 3200, 300, 300), Qt::AlignHCenter | Qt::AlignVCenter,
			"1");
		pPdfPainter->drawText(QRect(1820, 3200, 570, 300), Qt::AlignHCenter | Qt::AlignVCenter,
			"������Ϣ �ϸ���");
		pPdfPainter->drawText(QRect(50, 3200, 700, 300), Qt::AlignHCenter | Qt::AlignVCenter,
			QString("%1").arg("SR-ENS-600"));
		pPdfPainter->setFont(font[2]);




		delete pPdfPainter;
		delete pPdfWriter;
		pdfFile.close();
		//������ǰ·������Ϊԭ����·��


		tempDir.setCurrent(currentDir);
		delete m_pProgressBar;
		QMessageBox::warning(this, "��ʾ", "�������");
		setTipWindow("���ӿ����ӳ�ʱ����Ա������ɳɹ���");
	}
}
