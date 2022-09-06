#pragma once
#include <iostream>
#include "ui_bugRecords.h"
#include <qfiledialog.h>

#pragma execution_character_set("utf-8")
namespace generation_report
{

	Ui::bugRecordsClass ui;

	void CreateMasterAndSlavePdf(QString str = "./data/tele_acc.log")
	{
		//保存程序当前路径
		QString currentDir = QDir::currentPath();
		//生成pdf保存的路径
		QString file_path = QFileDialog::getExistingDirectory(this, "请选择保存路径", "C:\\Users\\Administrator\\Desktop");
		if (file_path.isEmpty())
		{
			return;
			QMessageBox::warning(this, "提示", "路径不能为空");
		}

		//将程序的执行路径设置到filePath下
		QDir tempDir;
		tempDir.setCurrent(file_path);

		//进度条
		QProgressBar* m_pProgressBar = new QProgressBar(this);
		m_pProgressBar->setOrientation(Qt::Horizontal);  // 水平方向
		m_pProgressBar->setMinimum(1);  // 最小值
		m_pProgressBar->setMaximum(1);  // 最大值 str.size()
		m_pProgressBar->setMaximumHeight(15);
		m_pProgressBar->setMinimumWidth(60);
		m_pProgressBar->setVisible(true);


		m_pProgressBar->setValue(0);
		QCoreApplication::processEvents();

		QDate date = QDate::currentDate();

		//QString serialNum = QString::number(Production.at(ui.toolComboBox->currentIndex()).second);
		//QString year = str.at(3).left(4);

		QString fileName = QString::fromStdString("主从操作准确度和重复性测试") + ".pdf";
		QFile pdfFile(fileName);
		//判断文件是否存在
		if (QFile::exists(fileName))
		{
			QMessageBox::StandardButton reply;
			reply = QMessageBox::question(this, "提示", "文件已经存在,点击是将覆盖原文件", QMessageBox::Yes | QMessageBox::No);
			if (reply == QMessageBox::Yes)
			{
				QFile::remove(fileName);
			}
			else
			{
				return;
			}
		}
		if (1)
		{
			// 打开要写入的pdf文件
			pdfFile.open(QIODevice::WriteOnly);

			QPdfWriter* pPdfWriter = new QPdfWriter(&pdfFile);  // 创建pdf写入器
			pPdfWriter->setPageSize(QPagedPaintDevice::A4);     // 设置纸张为A4
			pPdfWriter->setResolution(300);                     // 设置纸张的分辨率为300,因此其像素为3508X2479

			int iMargin = 0;                   // 页边距
			pPdfWriter->setPageMargins(QMarginsF(iMargin, iMargin, iMargin, iMargin));

			QPainter* pPdfPainter = new QPainter(pPdfWriter);   // qt绘制工具

			// 标题,居中
			QTextOption option(Qt::AlignHCenter | Qt::AlignVCenter);
			option.setWrapMode(QTextOption::WordWrap);

			//字体
			//QFont font;
			//font.setFamily("simhei.ttf");
			QFont font[8] = { QFont("宋体", 26, 60), QFont("黑体", 26, 61), QFont("宋体", 26, QFont::Normal), QFont("times new roman", 26, QFont::Normal), QFont("宋体", 26, QFont::Normal), QFont("华文新魏", 26, 61), QFont("黑体", 26, QFont::Normal), QFont("times new roman", 26, QFont::Normal) };
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
			//设置图片水平边距为150
			//pPdfPainter->setOpacity(0.4);
			//pPdfPainter->drawPixmap(5000, 100, 220 * 1.3, 110 * 1.3, pixmap);
			//pPdfPainter->drawPixmap(200, 100, 220 * 1.3, 110 * 1.3, pixmap);
			//pPdfPainter->setOpacity(1.0);


			//图纸大小
			int pdfWidth = 2478;
			int pdfHeight = 3508;
			pPdfPainter->setFont(font[2]);
			pPdfPainter->setPen(QPen(Qt::darkGray, 1));




			pPdfPainter->drawLine(150, 270, 2328, 270);  //2478-150

			pPdfPainter->setFont(font[5]);
			pPdfPainter->setPen(QPen(Qt::black, 2));
			pPdfPainter->drawText(QRect(0, 0, 2478 / 3, 250), Qt::AlignHCenter | Qt::AlignBottom,
				"北京术锐技术有限公司");


			//绘制臂体信息
			int chartW1 = 2478 - 400;
			int chartH1 = 950;
			int chartL1 = 200;
			int chartT1 = 550;
			int chartRowH1 = 85;
			int charti1 = 0;
			pPdfPainter->setFont(font[1]);
			pPdfPainter->setPen(QPen(Qt::black, 3));

			pPdfPainter->drawText(QRect(150, 300, 2328, 200), Qt::AlignHCenter | Qt::AlignVCenter,
				QString("%1报告").arg("主从操作准确度和重复性测试"));

			pPdfPainter->setFont(font[4]);
			pPdfPainter->setPen(QPen(Qt::black, 5));
			pPdfPainter->drawRect(chartL1, chartT1, chartW1, chartH1);
			pPdfPainter->setPen(QPen(Qt::black));
			pPdfPainter->setFont(font[2]);
			chartL1 += 15;
			pPdfPainter->drawText(QRect(chartL1 + 90, chartRowH1 * -1 + chartT1 + 12, pdfWidth / 4, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "工序 ：9527");
			pPdfPainter->drawText(QRect(pdfWidth / 2 + 340, chartRowH1 * -1 + chartT1 + 12, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "编号 ：         ");

			pPdfPainter->setFont(font[6]);
			pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("            %1").arg("内窥镜手术系统"));

			pPdfPainter->setFont(font[2]);
			pPdfPainter->drawText(QRect(pdfWidth / 2 + 260, chartRowH1 * charti1 + chartT1 + 10, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "测 试 员 ：         ");
			charti1++;
			pPdfPainter->drawLine(pdfWidth / 2 + 500, chartRowH1 * charti1 + chartT1 + 5, chartW1 + 100, chartRowH1 * charti1 + chartT1 + 5);

			pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("产品型号 ：%1").arg(QString::fromStdString("SR-ENS-600")));
			pPdfPainter->drawText(QRect(pdfWidth / 2 + 260, chartRowH1 * charti1 + chartT1 + 10, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "测试日期 ：         ");
			charti1++;
			pPdfPainter->drawLine(pdfWidth / 2 + 500, chartRowH1 * charti1 + chartT1 + 5, chartW1 + 100, chartRowH1 * charti1 + chartT1 + 5);
			pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("序 列 号 ：%1").arg("6002020000003"));
			pPdfPainter->drawText(QRect(pdfWidth / 2 + 260, chartRowH1 * charti1 + chartT1 + 10, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "审 核 员 ：         ");
			charti1++;
			pPdfPainter->drawLine(pdfWidth / 2 + 500, chartRowH1 * charti1 + chartT1 + 5, chartW1 + 100, chartRowH1 * charti1 + chartT1 + 5);
			pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("生产批号 ：%1").arg("20200003"));
			pPdfPainter->drawText(QRect(pdfWidth / 2 + 260, chartRowH1 * charti1 + chartT1 + 10, chartW1, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, "审核日期 ：         ");
			charti1++;
			pPdfPainter->drawLine(pdfWidth / 2 + 500, chartRowH1 * charti1 + chartT1 + 5, chartW1 + 100, chartRowH1 * charti1 + chartT1 + 5);
			//pPdfPainter->drawLine(chartL1 + 200, chartRowH1*charti1 + chartT1 + 10, chartW1 + 100, chartRowH1*charti1 + chartT1 + 10);


			pPdfPainter->setFont(font[6]);
			pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("            %1").arg("持针钳"));
			pPdfPainter->setFont(font[2]);
			charti1++;
			pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("产品型号 ：%1").arg(QString::fromStdString("SR-ENT-SP0807")));
			charti1++;
			pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("序 列 号 ：%1").arg(QString::fromStdString(" ")));
			charti1++;
			pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("生产批号 ：%1").arg(QString::fromStdString(" ")));
			charti1++;
			pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("设备名称 ：%1").arg(QString::fromStdString("主从操作准确度和重复性测试工装")));
			charti1++;
			pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("设备编号 ：%1").arg(QString::fromStdString(" ")));
			charti1++;
			pPdfPainter->drawText(QRect(chartL1 + 10, chartRowH1 * charti1 + chartT1 + 10, pdfWidth / 2, chartRowH1), Qt::AlignLeft | Qt::AlignVCenter, QString("检验标准 ：%1").arg(QString::fromStdString("《术锐 SR-ENS-600 技术要求》")));
			//pPdfPainter->drawLine(pdfWidth / 2 - 160, chartRowH1*charti1 + chartT1 + 10, chartW1 - 100, chartRowH1*charti1 + chartT1 + 10);
			pPdfPainter->setFont(font[2]);

			//读log
			qreal array[25][4] = { 0 };

			QFile file(str);

			if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
				return;
			}

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

			//表格——距离准确度测试结果
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
			//	"准确度和重复度测量");

			pPdfPainter->setPen(QPen(Qt::black, 3));

			pPdfPainter->setFont(font[3]);

			pPdfPainter->drawText(QRect(chartL2, chartT2, chartW2, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				" 距离准确度测试结果（单位：mm）");

			charti2++;
			pPdfPainter->drawText(QRect(chartL2, chartRowH2 * charti2 + chartT2, 278, chartRowH2 * 2), Qt::AlignHCenter | Qt::AlignVCenter,
				" 对角线 ");
			pPdfPainter->drawText(QRect(chartL2 + 278, chartRowH2 * charti2 + chartT2 + 5, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"距离准确度");
			pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 1, chartRowH2 * charti2 + chartT2 + 5, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"X方向距离准");
			pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 2, chartRowH2 * charti2 + chartT2 + 5, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"Y方向距离准");
			pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 3, chartRowH2 * charti2 + chartT2 + 5, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"Z方向距离准");

			charti2++;
			pPdfPainter->drawText(QRect(chartL2 + 278, chartRowH2 * charti2 + chartT2 - 5, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"|ADp|");
			pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 1, chartRowH2 * charti2 + chartT2 - 5, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"确度|ADx|");
			pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 2, chartRowH2 * charti2 + chartT2 - 5, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"确度|ADy|");
			pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 3, chartRowH2 * charti2 + chartT2 - 5, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				"确度|ADz|");

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

			//最大值
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
				" 最大值 ");
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
				" 要求 ");
			pPdfPainter->drawText(QRect(chartL2 + 278, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				" ≤ 2 ");
			pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 1, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				" ≤ 2 ");
			pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 2, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				" ≤ 2 ");
			pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 3, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				" ≤ 2 ");

			charti2++;
			pPdfPainter->drawText(QRect(chartL2, chartRowH2 * charti2 + chartT2, 278, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
				" 是否通过 ");
			if (biggest0 <= 2)
			{
				pPdfPainter->drawText(QRect(chartL2 + 278, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg("通过"));
			}

			else
			{
				pPdfPainter->drawText(QRect(chartL2 + 278, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg("未通过"));
			}

			if (biggest1 <= 2)
			{
				pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 1, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg("通过"));
			}

			else
			{
				pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 1, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg("未通过"));
			}

			if (biggest2 <= 2)
			{
				pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 2, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg("通过"));
			}

			else
			{
				pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 2, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg("未通过"));
			}
			if (biggest3 <= 2)
			{
				pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 3, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg("通过"));
			}

			else
			{
				pPdfPainter->drawText(QRect(chartL2 + 278 + 450 * 3, chartRowH2 * charti2 + chartT2, 450, chartRowH2), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg("未通过"));
			}


			//竖线
			pPdfPainter->drawLine(chartL2 + 278, chartT2 + chartRowH2, chartL2 + 278, chartT2 + chartH2);
			pPdfPainter->drawLine(chartL2 + 278 + 450 * 1, chartT2 + chartRowH2, chartL2 + 278 + 450 * 1, chartT2 + chartH2);
			pPdfPainter->drawLine(chartL2 + 278 + 450 * 2, chartT2 + chartRowH2, chartL2 + 278 + 450 * 2, chartT2 + chartH2);
			pPdfPainter->drawLine(chartL2 + 278 + 450 * 3, chartT2 + chartRowH2, chartL2 + 278 + 450 * 3, chartT2 + chartH2);
			//pPdfPainter->drawLine(chartL2 + 278 + 450 * 4, chartT2, chartL2 + 278 + 450 * 4, chartT2 + chartH2);
			//pPdfPainter->drawLine(chartL2 + 278 + 450 * 5, chartT2, chartL2 + 278 + 450 * 5, chartT2 + chartH2);

			//横线
			pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2, chartL2 + chartW2, chartT2 + chartRowH2);
			//pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 2, chartL2 + chartW2, chartT2 + chartRowH2 * 2);
			pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 3, chartL2 + chartW2, chartT2 + chartRowH2 * 3);
			pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 4, chartL2 + chartW2, chartT2 + chartRowH2 * 4);
			pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 5, chartL2 + chartW2, chartT2 + chartRowH2 * 5);
			pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 6, chartL2 + chartW2, chartT2 + chartRowH2 * 6);
			pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 7, chartL2 + chartW2, chartT2 + chartRowH2 * 7);
			pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 8, chartL2 + chartW2, chartT2 + chartRowH2 * 8);
			pPdfPainter->drawLine(chartL2, chartT2 + chartRowH2 * 9, chartL2 + chartW2, chartT2 + chartRowH2 * 9);




			//表格——距离重复性测试结果
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
			//	"准确度和重复度测量");

			pPdfPainter->setPen(QPen(Qt::black, 3));

			pPdfPainter->setFont(font[3]);

			pPdfPainter->drawText(QRect(chartL3, chartT3, chartW3, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				" 距离重复性测试结果（单位：mm）");

			charti3++;
			pPdfPainter->drawText(QRect(chartL3, chartRowH3 * charti3 + chartT3, 278, chartRowH3 * 2), Qt::AlignHCenter | Qt::AlignVCenter,
				" 对角线 ");
			pPdfPainter->drawText(QRect(chartL3 + 278, chartRowH3 * charti3 + chartT3 + 5, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				"距离重复性");
			pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 1, chartRowH3 * charti3 + chartT3 + 5, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				"X方向距离准");
			pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 2, chartRowH3 * charti3 + chartT3 + 5, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				"Y方向距离准");
			pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 3, chartRowH3 * charti3 + chartT3 + 5, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				"Z方向距离准");

			charti3++;
			pPdfPainter->drawText(QRect(chartL3 + 278, chartRowH3 * charti3 + chartT3 - 5, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				"RD(±)");
			pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 1, chartRowH3 * charti3 + chartT3 - 5, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				"确度RDx(±)");
			pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 2, chartRowH3 * charti3 + chartT3 - 5, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				"确度RDy(±)");
			pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 3, chartRowH3 * charti3 + chartT3 - 5, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				"确度RDz(±)");

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

			//最大值
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
				" 最大值 ");
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
				" 要求 ");
			pPdfPainter->drawText(QRect(chartL3 + 278, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				" ≤ 2 ");
			pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 1, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				" ≤ 2 ");
			pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 2, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				" ≤ 2 ");
			pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 3, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				" ≤ 2 ");

			charti3++;
			pPdfPainter->drawText(QRect(chartL3, chartRowH3 * charti3 + chartT3, 278, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
				" 是否通过 ");
			if (biggest4 <= 2)
			{
				pPdfPainter->drawText(QRect(chartL3 + 278, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg("通过"));
			}

			else
			{
				pPdfPainter->drawText(QRect(chartL3 + 278, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg("未通过"));
			}

			if (biggest5 <= 2)
			{
				pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 1, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg("通过"));
			}

			else
			{
				pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 1, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg("未通过"));
			}

			if (biggest6 <= 2)
			{
				pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 2, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg("通过"));
			}

			else
			{
				pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 2, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg("未通过"));
			}
			if (biggest7 <= 2)
			{
				pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 3, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg("通过"));
			}

			else
			{
				pPdfPainter->drawText(QRect(chartL3 + 278 + 450 * 3, chartRowH3 * charti3 + chartT3, 450, chartRowH3), Qt::AlignHCenter | Qt::AlignVCenter,
					QString(" %1 ").arg("未通过"));
			}


			//竖线
			pPdfPainter->drawLine(chartL3 + 278, chartT3 + chartRowH3, chartL3 + 278, chartT3 + chartH3);
			pPdfPainter->drawLine(chartL3 + 278 + 450 * 1, chartT3 + chartRowH3, chartL3 + 278 + 450 * 1, chartT3 + chartH3);
			pPdfPainter->drawLine(chartL3 + 278 + 450 * 2, chartT3 + chartRowH3, chartL3 + 278 + 450 * 2, chartT3 + chartH3);
			pPdfPainter->drawLine(chartL3 + 278 + 450 * 3, chartT3 + chartRowH3, chartL3 + 278 + 450 * 3, chartT3 + chartH3);
			//pPdfPainter->drawLine(chartL3 + 278 + 450 * 4, chartT3, chartL3 + 278 + 450 * 4, chartT3 + chartH3);
			//pPdfPainter->drawLine(chartL3 + 278 + 450 * 5, chartT3, chartL3 + 278 + 450 * 5, chartT3 + chartH3);

			//横线
			pPdfPainter->drawLine(chartL3, chartT3 + chartRowH3, chartL3 + chartW3, chartT3 + chartRowH3);
			//pPdfPainter->drawLine(chartL3, chartT3 + chartRowH3 * 2, chartL3 + chartW3, chartT3 + chartRowH3 * 2);
			pPdfPainter->drawLine(chartL3, chartT3 + chartRowH3 * 3, chartL3 + chartW3, chartT3 + chartRowH3 * 3);
			pPdfPainter->drawLine(chartL3, chartT3 + chartRowH3 * 4, chartL3 + chartW3, chartT3 + chartRowH3 * 4);
			pPdfPainter->drawLine(chartL3, chartT3 + chartRowH3 * 5, chartL3 + chartW3, chartT3 + chartRowH3 * 5);
			pPdfPainter->drawLine(chartL3, chartT3 + chartRowH3 * 6, chartL3 + chartW3, chartT3 + chartRowH3 * 6);
			pPdfPainter->drawLine(chartL3, chartT3 + chartRowH3 * 7, chartL3 + chartW3, chartT3 + chartRowH3 * 7);
			pPdfPainter->drawLine(chartL3, chartT3 + chartRowH3 * 8, chartL3 + chartW3, chartT3 + chartRowH3 * 8);
			pPdfPainter->drawLine(chartL3, chartT3 + chartRowH3 * 9, chartL3 + chartW3, chartT3 + chartRowH3 * 9);


			//页脚
			pPdfPainter->setFont(font[3]);
			pPdfPainter->setPen(QPen(Qt::darkGray, 1));
			pPdfPainter->drawLine(150, 3310, 2328, 3310);  //2478-150

			pPdfPainter->drawText(QRect(1100, 3200, 300, 300), Qt::AlignHCenter | Qt::AlignVCenter,
				"1/2");
			pPdfPainter->drawText(QRect(1820, 3200, 570, 300), Qt::AlignHCenter | Qt::AlignVCenter,
				"机密信息 严格保密");
			pPdfPainter->drawText(QRect(50, 3200, 700, 300), Qt::AlignHCenter | Qt::AlignVCenter,
				QString("%1").arg("SR-ENS-600"));
			pPdfPainter->setFont(font[2]);



			//第二页
			pPdfWriter->newPage();

			/*		pPdfPainter->setOpacity(0.4);
					pPdfPainter->drawPixmap(200, 100, 220 * 1.3, 110 * 1.3, pixmap);
					pPdfPainter->setOpacity(1.0);
					pPdfPainter->setFont(font[2])*/;
					pPdfPainter->setPen(QPen(Qt::darkGray, 1));
					//pPdfPainter->drawText(QRect(0, 0, 2478/2, 250), Qt::AlignHCenter | Qt::AlignBottom,
					//	"北京术锐技术有限公司");
					//pPdfPainter->drawText(QRect(1500, 0, 2478 - 1500, 250), Qt::AlignHCenter | Qt::AlignBottom,
					//	QString("文件编号 ：%1").arg(str.at(2)));
					pPdfPainter->drawLine(150, 270, 2328, 270);  //2478-150

					pPdfPainter->setFont(font[5]);
					pPdfPainter->setPen(QPen(Qt::black, 2));
					pPdfPainter->drawText(QRect(0, 0, 2478 / 3, 250), Qt::AlignHCenter | Qt::AlignBottom,
						"北京术锐技术有限公司");

					pPdfPainter->setPen(QPen(Qt::black, 1));
					pPdfPainter->setFont(font[2]);


					//表格——姿态准确度测试结果
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
					//	"姿态准确度测试结果");

					pPdfPainter->setPen(QPen(Qt::black, 3));

					pPdfPainter->setFont(font[3]);

					pPdfPainter->drawText(QRect(chartL4, chartT4, chartW4, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
						" 姿态准确度测试结果（单位：°）");

					charti4++;
					pPdfPainter->drawText(QRect(chartL4, chartRowH4 * charti4 + chartT4, 278, chartRowH4 * 2), Qt::AlignHCenter | Qt::AlignVCenter,
						" 点位 ");
					pPdfPainter->drawText(QRect(chartL4 + 278, chartRowH4 * charti4 + chartT4 + 5, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
						"欧拉角分量a准确度");
					pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 1, chartRowH4 * charti4 + chartT4 + 5, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
						"欧拉角分量b准确度");
					pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 2, chartRowH4 * charti4 + chartT4 + 5, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
						"欧拉角分量c准确度");


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

					//最大值
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
						" 最大值 ");
					pPdfPainter->drawText(QRect(chartL4 + 278, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
						QString(" %1 ").arg(QString::number(biggest8, 'f', 2)));
					pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 1, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
						QString(" %1 ").arg(QString::number(biggest9, 'f', 2)));
					pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 2, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
						QString(" %1 ").arg(QString::number(biggest10, 'f', 2)));

					charti4++;
					pPdfPainter->drawText(QRect(chartL4, chartRowH4 * charti4 + chartT4, 278, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
						" 要求 ");
					pPdfPainter->drawText(QRect(chartL4 + 278, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
						"≤ 3");
					pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 1, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
						"≤ 3");
					pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 2, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
						"≤ 3");

					charti4++;
					pPdfPainter->drawText(QRect(chartL4, chartRowH4 * charti4 + chartT4, 278, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
						" 是否通过 ");
					if (biggest8 <= 3)
					{
						pPdfPainter->drawText(QRect(chartL4 + 278, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
							"通过");
					}

					else
					{
						pPdfPainter->drawText(QRect(chartL4 + 278, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
							"未通过");
					}

					if (biggest9 <= 3)
					{
						pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 1, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
							"通过");
					}

					else
					{
						pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 1, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
							"未通过");
					}

					if (biggest10 <= 3)
					{
						pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 2, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
							"通过");
					}

					else
					{
						pPdfPainter->drawText(QRect(chartL4 + 278 + 600 * 2, chartRowH4 * charti4 + chartT4, 600, chartRowH4), Qt::AlignHCenter | Qt::AlignVCenter,
							"未通过");
					}





					//竖线
					pPdfPainter->drawLine(chartL4 + 278, chartT4 + chartRowH4, chartL4 + 278, chartT4 + chartH4);
					pPdfPainter->drawLine(chartL4 + 278 + 600 * 1, chartT4 + chartRowH4, chartL4 + 278 + 600 * 1, chartT4 + chartH4);
					pPdfPainter->drawLine(chartL4 + 278 + 600 * 2, chartT4 + chartRowH4, chartL4 + 278 + 600 * 2, chartT4 + chartH4);
					//pPdfPainter->drawLine(chartL4 + 278 + 600 * 3, chartT4 + chartRowH4, chartL4 + 278 + 600 * 3, chartT4 + chartH4);
					//pPdfPainter->drawLine(chartL4 + 278 + 450 * 4, chartT4, chartL4 + 278 + 450 * 4, chartT4 + chartH4);
					//pPdfPainter->drawLine(chartL4 + 278 + 450 * 5, chartT4, chartL4 + 278 + 450 * 5, chartT4 + chartH4);

					//横线
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



					//表格——姿态重复性测试结果
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
					//	"姿态重复性测试结果");

					pPdfPainter->setPen(QPen(Qt::black, 3));

					pPdfPainter->setFont(font[3]);

					pPdfPainter->drawText(QRect(chartL5, chartT5, chartW5, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
						" 姿态重复性测试结果（单位：°）");

					charti5++;
					pPdfPainter->drawText(QRect(chartL5, chartRowH5 * charti5 + chartT5, 278, chartRowH5 * 2), Qt::AlignHCenter | Qt::AlignVCenter,
						" 点位 ");
					pPdfPainter->drawText(QRect(chartL5 + 278, chartRowH5 * charti5 + chartT5 + 5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
						"欧拉角分量a重复性");
					pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 1, chartRowH5 * charti5 + chartT5 + 5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
						"欧拉角分量b重复性");
					pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 2, chartRowH5 * charti5 + chartT5 + 5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
						"欧拉角分量c重复性");


					charti5++;
					pPdfPainter->drawText(QRect(chartL5 + 278, chartRowH5 * charti5 + chartT5 - 5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
						"RPa(±)");
					pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 1, chartRowH5 * charti5 + chartT5 - 5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
						"RPb(±)");
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

					//最大值
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
						" 最大值 ");
					pPdfPainter->drawText(QRect(chartL5 + 278, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
						QString(" %1 ").arg(QString::number(biggest11, 'f', 2)));
					pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 1, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
						QString(" %1 ").arg(QString::number(biggest12, 'f', 2)));
					pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 2, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
						QString(" %1 ").arg(QString::number(biggest13, 'f', 2)));


					charti5++;
					pPdfPainter->drawText(QRect(chartL5, chartRowH5 * charti5 + chartT5, 278, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
						" 要求 ");
					pPdfPainter->drawText(QRect(chartL5 + 278, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
						"≤ 2");
					pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 1, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
						"≤ 2");
					pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 2, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
						"≤ 2");

					charti5++;
					pPdfPainter->drawText(QRect(chartL5, chartRowH5 * charti5 + chartT5, 278, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
						" 是否通过 ");
					if (biggest11 <= 2)
					{
						pPdfPainter->drawText(QRect(chartL5 + 278, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
							"通过");
					}

					else
					{
						pPdfPainter->drawText(QRect(chartL5 + 278, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
							"未通过");
					}

					if (biggest12 <= 2)
					{
						pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 1, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
							"通过");
					}

					else
					{
						pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 1, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
							"未通过");
					}


					if (biggest13 <= 2)
					{
						pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 2, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
							"通过");
					}

					else
					{
						pPdfPainter->drawText(QRect(chartL5 + 278 + 600 * 2, chartRowH5 * charti5 + chartT5, 600, chartRowH5), Qt::AlignHCenter | Qt::AlignVCenter,
							"未通过");
					}






					//竖线
					pPdfPainter->drawLine(chartL5 + 278, chartT5 + chartRowH5, chartL5 + 278, chartT5 + chartH5);
					pPdfPainter->drawLine(chartL5 + 278 + 600 * 1, chartT5 + chartRowH5, chartL5 + 278 + 600 * 1, chartT5 + chartH5);
					pPdfPainter->drawLine(chartL5 + 278 + 600 * 2, chartT5 + chartRowH5, chartL5 + 278 + 600 * 2, chartT5 + chartH5);
					//pPdfPainter->drawLine(chartL5 + 278 + 600 * 3, chartT5 + chartRowH5, chartL5 + 278 + 600 * 3, chartT5 + chartH5);
					//pPdfPainter->drawLine(chartL5 + 278 + 450 * 4, chartT5, chartL5 + 278 + 450 * 4, chartT5 + chartH5);
					//pPdfPainter->drawLine(chartL5 + 278 + 450 * 5, chartT5, chartL5 + 278 + 450 * 5, chartT5 + chartH5);

					//横线
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





					//页脚
					pPdfPainter->setFont(font[3]);
					pPdfPainter->setPen(QPen(Qt::darkGray, 1));
					pPdfPainter->drawLine(150, 3310, 2328, 3310);  //2478-150

					pPdfPainter->drawText(QRect(1100, 3200, 300, 300), Qt::AlignHCenter | Qt::AlignVCenter,
						"2/2");
					pPdfPainter->drawText(QRect(1820, 3200, 570, 300), Qt::AlignHCenter | Qt::AlignVCenter,
						"机密信息 严格保密");
					pPdfPainter->drawText(QRect(50, 3200, 700, 300), Qt::AlignHCenter | Qt::AlignVCenter,
						QString("%1").arg("SR-ENS-600"));
					pPdfPainter->setFont(font[2]);




					delete pPdfPainter;
					delete pPdfWriter;
					pdfFile.close();
					//将程序当前路径设置为原来的路径


					tempDir.setCurrent(currentDir);
					delete m_pProgressBar;
					QMessageBox::warning(this, "提示", "导出完成");
		}
	}


}