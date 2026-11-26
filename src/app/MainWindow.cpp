#include "MainWindow.h"
#include "PidActorManager.h"

#include <algorithm>
#include <cmath>

#include <QAction>
#include <QScrollBar>
#include <QActionGroup>
#include <QMenuBar>
#include <QComboBox>
#include <QDateTime>
#include <QDesktopServices>
#include <QSignalBlocker>
#include <QDir>
#include <QUrl>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QFile>
#include <QKeyEvent>
#include <QLabel>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QMenu>
#include <QMenuBar>
#include <QStandardPaths>
#include <QStatusBar>
#include <QTextBrowser>
#include <QToolBar>
#include <QTreeWidget>
#include <QTreeWidgetItem>

#include <QVTKOpenGLNativeWidget.h>

#include <vtkActor.h>
#include <vtkAppendPolyData.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkCellPicker.h>
#include <vtkCommand.h>
#include <vtkCoordinate.h>
#include <vtkCubeSource.h>
#include <vtkFloatArray.h>
#include <functional>
#include <vtkActor2D.h>
#include <vtkPolyDataMapper2D.h>
#include <vtkToneMappingPass.h>
#include <vtkPointData.h>
#include <vtkGlyph3D.h>
#include <vtkDoubleArray.h>
#include <random>
#include <vtkParametricFunctionSource.h>
#include <vtkParametricTorus.h>
#include <vtkDataArray.h>
#include <vtkDataSetMapper.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkImageData.h>
#include <vtkInteractorStyleUser.h>
#include <vtkLight.h>
#include <vtkLinearTransform.h>
#include <vtkLookupTable.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkPNGWriter.h>
#include <vtkPlaneSource.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRegularPolygonSource.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkScalarBarActor.h>
#include <vtkUnsignedCharArray.h>
#include "core/VehicleShapes.h"
#include <vtkSkybox.h>
#include <vtkSphereSource.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkTexture.h>
#include <vtkTransform.h>
#include <vtkUnstructuredGrid.h>
#include <vtkWindowToImageFilter.h>
#include <vtkRenderStepsPass.h>
#include <vtkSSAOPass.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkIdList.h>
#include <vtkTriangleFilter.h>
#include <vtkShaderProperty.h>

#include "core/Palette.h"
#include "core/SceneBuilder.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;

QString ko(const char* utf8) { return QString::fromUtf8(utf8); }

void lerp3(double* a, const double* b, double k)
{
    for (int i = 0; i < 3; ++i)
        a[i] += (b[i] - a[i]) * k;
}

vtkSmartPointer<vtkTextActor> makeText(double x, double y, int size, bool top)
{
    auto t = vtkSmartPointer<vtkTextActor>::New();
    t->GetPositionCoordinate()->SetCoordinateSystemToNormalizedViewport();
    t->SetPosition(x, y);
    vtkTextProperty* p = t->GetTextProperty();
    p->SetFontFamilyToArial();
    p->SetFontSize(size);
    p->SetColor(0.93, 0.96, 1.0);
    p->SetLineSpacing(1.15);
    if (top)
        p->SetVerticalJustificationToTop();
    return t;
}

// Korean user guide shown in the right dock (F1)
const char* kGuideHtml = u8R"(
<h2>LiDAR Drive 사용 안내</h2>
<p>차(툴바 <b>차종</b>: 트럭·세단·SUV·스포츠카)가 <b>왕복 6차선, 한 바퀴 약 2.2 km의 순환 코스</b>를 달리는 시뮬레이터입니다. <b>도심 한가운데서 출발</b>합니다. 직접 운전하거나 <b>자율주행</b>을 켜서
LiDAR 센서만으로 장애물과 다른 차를 피하는 모습을 볼 수 있고, 부딪히면 차체가 <b>찌그러집니다</b>.</p>

<h3>🚀 빠른 시작</h3>
<ol>
<li><b>W</b>를 누르면 출발합니다.</li>
<li><b>T</b> (또는 툴바 <i>자율주행</i>)를 누르면 차가 스스로 운전합니다.</li>
<li><b>C</b>로 카메라 시점을 바꿔 보세요.</li>
<li>일부러 부딪혀 보고, <b>R</b>로 처음부터 다시 시작합니다.</li>
<li><b>F9</b>로 <b>데이터 녹화</b>를 켜면 LiDAR·카메라 데이터셋이 저장됩니다.</li>
</ol>

<h3>⌨️ 조작</h3>
<table border="0" cellspacing="2" cellpadding="2">
<tr><td><b>W / ↑</b></td><td>가속 (차종마다 다름: 트럭 0→100 km/h 약 10초 ~ 스포츠카 약 5초)</td></tr>
<tr><td><b>Q / E</b></td><td>기어 내림 / 올림 (누르면 수동 변속 M 모드)</td></tr>
<tr><td><b>M</b></td><td>자동(D) ↔ 수동(M) 변속</td></tr>
<tr><td><b>S / ↓</b></td><td>브레이크 (멈춰 있으면 후진)</td></tr>
<tr><td><b>A D / ← →</b></td><td>핸들</td></tr>
<tr><td><b>Space</b></td><td>사이드 브레이크 (뒷바퀴 잠김 → 뒤가 미끄러짐)</td></tr>
<tr><td><b>X</b></td><td>주행 보조(ABS·TCS·ESC) 끄기/켜기 — 끄면 드리프트 가능</td></tr>
<tr><td><b>T</b></td><td>자율주행 켜기/끄기 (큰 순환도로에서만)</td></tr>
<tr><td><b>1 2 3 4</b></td><td>차종 바꾸기: 1 트럭 · 2 세단 · 3 SUV · 4 스포츠카 (맨 위 메뉴 <b>🚗 차종 바꾸기</b>도 같음)</td></tr>
<tr><td><b>K</b></td><td>손상(변형률) 색 계속 보기 / 충돌 직후 4초만</td></tr>
<tr><td><b>C</b></td><td>카메라 바꾸기</td></tr>
<tr><td><b>R</b></td><td>리셋 (방금 주행이 고스트로 남음)</td></tr>
<tr><td><b>Enter</b></td><td>일시정지 / 재개</td></tr>
<tr><td><b>L / G</b></td><td>LiDAR 점 / 고스트 표시</td></tr>
<tr><td><b>F1</b></td><td>이 안내 열고 닫기</td></tr>
<tr><td><b>F9</b></td><td><b>데이터 녹화 시작/정지</b></td></tr>
<tr><td><b>F12</b></td><td>스크린샷 저장 (사진 폴더)</td></tr>
<tr><td><b>마우스 드래그</b></td><td>카메라 회전 (주행 중에도 가능)</td></tr>
<tr><td><b>마우스 휠</b></td><td>확대 / 축소</td></tr>
<tr><td><b>V</b></td><td><b>1인칭(운전석) ↔ 3인칭 전환</b></td></tr>
<tr><td><b>더블클릭 / Home</b></td><td>시점 원위치</td></tr>
<tr><td><b>클릭</b></td><td>부품(PID) 선택 → <b>노란 선</b>으로 강조 (원본 예제 기능)</td></tr>
<tr><td><b>Esc</b></td><td>선택 해제 (노란 선 끄기)</td></tr>
</table>

<h3>🖥️ 화면 구성</h3>
<ul>
<li><b>왼쪽 위 글자판(HUD)</b>: 속도, 핸들 각도, 현재 상태, 주행 거리, LiDAR 성능</li>
<li><b>오른쪽 위 FRONT CAMERA</b>: 차 지붕 카메라로 본 앞쪽 화면</li>
<li><b>HUD의 SIGNAL</b>: 다음 신호등의 색, 정지선까지 거리, 바뀌기까지 남은 초</li>
<li><b>오른쪽 아래 LIDAR TOP VIEW</b>: LiDAR가 감지한 점만 위에서 본 화면 (원은 10 m 간격)</li>
<li><b>왼쪽 아래 MAP</b>: 순환 코스 전체(가운데 호수)와 내 위치(하늘색 점), 다른 차(흰 점)</li>
<li><b>아래 상태 표시줄</b>: 지금 어느 구간인지, 자율주행이 무엇을 하는지 한글로 설명</li>
</ul>

<h3>🗺️ 코스 (왕복 6차선, 우측통행)</h3>
<p>가운데 <span style="color:#db1"><b>노란 이중 중앙선</b></span>을 기준으로 오른쪽 3차선이 내 방향,
왼쪽 3차선은 <b>마주 오는 차</b>가 달리는 반대 방향입니다. 차선 사이는 흰 점선.</p>
<p>끝이 없는 <b>순환 도로(시계 방향)</b>라 계속 달릴 수 있습니다. 직선은 도심 하나뿐이고 나머지는 곡선입니다.</p>
<ol>
<li><b>도심 (출발)</b>: 420 m 직선, 교차로 6곳과 왕복 4차선 옆길, 횡단보도·정지선, 차선마다 신호등,
    인도·가로수·가로등, 유리 고층 빌딩, 옆길에 주차된 차, <b>같은 방향 차 + 마주 오는 차</b>.
    옆길은 양쪽 78 m 뒤의 <b>뒷길</b>과 T자로 만나서, 좌·우회전해도 길이 끊기지 않고 블록을 돌아 나올 수 있습니다.
    건물은 4줄 (큰길 앞, 그 뒤, 뒷길 앞, 뒷길 건너편 고층)</li>
<li><b>코너 4곳</b> (반지름 110 m, 90° 우회전): 바깥쪽 가드레일과 노란 화살표 표지. 자율주행은 미리 감속</li>
<li><b>장애물 코스 (숲길)</b>: S자 곡선 도로, 울창한 침엽수 숲. 문마다 3차선 중 한 차선만 열려 있어 차선을 바꿔 가며 통과</li>
<li><b>고속도로</b>: 길게 휘어지는 S자, 양쪽 가드레일과 콘크리트 중앙분리대, 가로등, 초록 안내판, 빠른 양방향 교통.
    가운데에 산을 뚫은 <b>190 m 쌍굴 터널</b> (방향마다 굴 하나, 타일 벽·아치 천장·천장 조명, 안은 어둡게)</li>
<li><b>교외 주택가</b>: 박공지붕 주택 두 줄, 정원수, 인도, 가로등, <b>주유소</b>(캐노피·주유기·편의점),
    바깥쪽 언덕에 <b>아파트 단지</b></li>
<li>주변: 코스 안쪽엔 <b>호수와 공원</b>(요트, 나무 잔교, 정자)과 <b>논·밭·비닐하우스</b>,
    바깥쪽엔 <b>산</b>(능선 위 <b>풍력발전기</b>), 고속도로 바깥을 따라 <b>송전탑과 전선</b>, 코너마다 <b>광고판</b>
    (LiDAR도 산·건물·비닐하우스를 감지)</li>
<li><b>도로망</b> (약 2 km, 왕복 6차선): 도심 출발 후 첫 번째 교차로에서 우회전해서 뒷길을 건너 계속 직진하면
    T자 교차로로 만납니다. 호숫가를 지나 <b>고가도로</b>로 고속도로 위를 넘고(높이 7.6 m, 교각·난간),
    산길을 오르내린 뒤(최대 경사 6.5%) 교외 도로 아래를 <b>지하차도</b>(옹벽, 덮개 아래 조명)로 지나 다시 돌아옵니다.
    차는 도로 높이와 경사를 따라 기울어지고, 이곳에도 다른 차가 다닙니다. 툴바 <b>출발</b>에서
    "도로망 · 고가도로 앞 / 지하차도 앞"을 고르면 바로 갈 수 있습니다 (자율주행은 큰 순환도로 전용)</li>
</ol>
)" u8R"(<h3>🚦 신호등</h3>
<p>도심 교차로 6곳의 신호등은 <b>교차로마다 주기(34~46초)와 시작 시점이 달라서</b> 따로따로 바뀝니다.
한국식 가로형 4구 신호등: <span style="color:#e33">■ 빨강</span> <span style="color:#db1">■ 노랑</span>
<span style="color:#3c5">← 좌회전 화살표</span> <span style="color:#3c5">■ 초록</span>.</p>
<ul>
<li>순서: 직진 초록 → 노랑(3초) → 빨강 + <b>좌회전 화살표</b>(6초) → 전체 빨강 → <b>옆길 초록</b> → 옆길 노랑 → 다시 직진 초록</li>
<li>기둥 아래쪽 작은 신호등은 옆길용 (직진 신호와 반대로 켜짐)</li>
<li><b>다른 차</b>는 빨간불에 정지선 앞에서 멈추고, 노란불은 멈출 수 있을 때만 멈춥니다 (딜레마 구간 판단)</li>
<li><b>자율주행</b>은 지도(V2X)로 신호를 받아 정지선 1 m 앞에 부드럽게 섭니다 (LiDAR는 색을 못 보므로)</li>
<li>직접 운전할 때 빨간불에 정지선을 넘으면 <b>RED LIGHT VIOLATION</b>이 뜨고 횟수가 셉니다</li>
</ul>

<h3>⚙️ 8단 자동변속기</h3>
<p>엔진 회전수(rpm)에 따라 토크가 달라지는 엔진 곡선과 <b>8단 변속기</b>(기어비 4.71 → 0.67), 토크 컨버터가 들어 있습니다.
HUD의 <b>GEAR</b> 줄에 현재 단(D3 / 8), rpm 막대, 엔진 토크가 나옵니다.</p>
<ul>
<li><b>자동(D)</b>: 살살 밟으면 일찍 올리고(연비), 꾹 밟으면 높은 rpm까지 끌고 가거나 <b>킥다운</b>으로 내립니다</li>
<li><b>수동(M)</b>: <b>Q</b> 내림 / <b>E</b> 올림. 레드존까지 돌리면 리미터가 걸리고, 너무 낮은 rpm이면 저절로 내립니다</li>
<li>변속하는 0.2초 동안 힘이 잠깐 줄어듭니다 (SHIFT 표시). 가속 페달을 떼면 엔진 브레이크</li>
<li>자율주행은 항상 자동(D)으로 달립니다</li>
<li><b>기어비가 힘을 결정</b>합니다: 바퀴 구동력 = 엔진 토크 × 기어비 × 종감속비 ÷ 바퀴 반지름.
    1단은 8단보다 7배 세게 밀고, 타이어 접지 한계를 넘으면 휠스핀(TCS가 잡음)</li>
</ul>

<h3>🛞 차량 동역학</h3>
<ul>
<li><b>2자유도 단일 트랙(bicycle) 모델</b>: 앞·뒤 타이어 슬립각 → 매직 포뮬러 횡력, 요 관성으로 회전</li>
<li><b>하중 이동</b>: 가속하면 뒤로, 제동하면 앞으로 무게가 쏠려 바퀴별 접지력이 바뀝니다 (무게중심 높은 트럭일수록 큼)</li>
<li><b>마찰원</b>: 제동·구동에 쓴 만큼 코너링 그립이 줄어듭니다. 비포장(잔디)은 마찰계수 0.6</li>
<li><b>ABS · EBD · TCS · ESC</b>: 바퀴 잠김/헛돎 방지, 하중 비례 제동 배분, 스핀 억제 (요즘 차와 같음).
    ESC는 미끄러지기 시작하면 바퀴 하나를 잡고 엔진 출력도 줄입니다</li>
<li><b>드리프트</b>: <b>X</b>로 주행 보조를 끄고 코너에서 풀 가속(후륜구동이라 뒤가 밀려남), 또는 <b>Space</b>(사이드 브레이크)로
    뒷바퀴를 잠가 뒤를 날린 뒤 반대로 핸들(카운터 스티어). 스포츠카가 가장 쉽습니다. 자율주행 중에는 보조가 항상 켜짐</li>
<li><b>다른 차도 같은 동역학</b>: 출력/무게로 정해지는 가속, 코너에서 롤·제동 시 피치. 부딪히면 차선을 벗어나
    미끄러지고 회전하다가 타이어 마찰로 멈춥니다 (충격량 + 각운동량)</li>
<li><b>바퀴·차축 손상</b>: 바퀴 근처를 받으면 바퀴가 안쪽으로 밀리고 기울어짐(캠버·토 틀어짐) → 접지력 감소,
    핸들이 한쪽으로 쏠림. 더 세게 받으면 차축·서스펜션이 휘고, 아주 세면 <b>바퀴가 빠져</b> 그 모서리가 주저앉고 허브가 바닥에
    끌립니다. 충돌 분석 탭의 <b>바퀴 · 차축</b>에서 확인</li>
<li>차체는 코너에서 바깥으로 <b>롤</b>, 가속 시 앞이 들리고 제동 시 앞이 숙여집니다(<b>피치</b>). F3 정보에서 슬립각·횡가속도 확인</li>
</ul>

<h3>🚗 차종</h3>
<p>숫자키 <b>1~4</b>, 맨 위 메뉴 <b>🚗 차종 바꾸기</b>, 또는 툴바 <b>차종</b>에서 고르면
모양·무게·엔진 출력·브레이크·LiDAR와 카메라 높이가 모두 바뀝니다.
모두 원본 예제와 같은 차량 좌표계(PID 1~4)를 써서 충돌 변형·충돌 리포트·LiDAR가 그대로 동작합니다.</p>
<table border="0" cellspacing="2" cellpadding="2">
<tr><td><b>트럭</b></td><td>3.5톤, 190 kW / 709 Nm 디젤 (레드존 3800 rpm), 0→100 10.3초, 최고 144 km/h, LiDAR 높이 1.62 m</td></tr>
<tr><td><b>세단</b></td><td>1.55톤, 135 kW / 278 Nm (6500 rpm), 0→100 6.5초, 최고 209 km/h</td></tr>
<tr><td><b>SUV</b></td><td>2.2톤, 185 kW / 425 Nm (6000 rpm), 0→100 6.6초, 최고 187 km/h, 높은 차체</td></tr>
<tr><td><b>스포츠카</b></td><td>1.45톤, 330 kW / 563 Nm (7800 rpm), 0→100 4.7초, 최고 288 km/h, 낮은 차체 (LiDAR도 낮아짐)</td></tr>
</table>

<p>툴바 <b>출발</b>에서 도심 / 장애물 코스 / 고속도로 / 교외 주택가 / 도로망(고가·지하차도 앞) 중 출발 위치를
고를 수 있습니다. 다른 차는 구간 제한속도(도심·교외 50, 숲길·코너 60, 고속도로 100, 도로망 70 km/h)에
사람마다 조금씩 다른 속도로 달리고, 지나간 차는 앞쪽에 다시 나타나서 교통이 계속 이어집니다.</p>

<h3>🛰️ LiDAR (빛으로 거리를 재는 센서)</h3>
<p>지붕 위 센서는 실제 <b>Velodyne Ultra Puck (VLP-32C)</b> 스펙으로 만들었습니다.</p>
<table border="0" cellspacing="2" cellpadding="2">
<tr><td>채널</td><td><b>32개</b> (세로 -25° ~ +15°, 지평선 근처 0.33° 간격으로 촘촘)</td></tr>
<tr><td>회전</td><td><b>10 Hz</b>, 가로 해상도 <b>0.2°</b> → 한 바퀴 1,800방향</td></tr>
<tr><td>점 개수</td><td>한 바퀴 57,600개 레이저, <b>초당 약 58만 점</b></td></tr>
<tr><td>거리</td><td>최대 <b>200 m</b>, 정확도 ±3 cm</td></tr>
</table>
<p>실제 센서처럼 <b>빙글빙글 돌면서</b> 지나간 방향만 새로 찍습니다 (10 ms마다 36°씩).</p>
<ul>
<li><b>점 색 = 반사 강도(intensity)</b>:
 <span style="color:#24d">파랑(약함)</span> → <span style="color:#1bf">청록</span> → <span style="color:#3c5">초록</span> →
 <span style="color:#db1">노랑</span> → <span style="color:#e33">빨강(강함)</span></li>
<li>아스팔트·유리는 약하고, 잔디·건물은 중간, <b>차선 페인트·횡단보도·표지판·라이트·드럼통 반사띠</b>는
 빛을 되돌려 보내는 재귀반사 재질이라 아주 강합니다 → 바닥 점에 <b>차선이 밝게 떠오릅니다</b></li>
<li>레이저가 표면에 비스듬히 맞을수록 약해집니다 (입사각). 재질마다 반사 방식이 다릅니다:
 <b>난반사</b>(콘크리트·잔디, 각도에 비례) / <b>재귀반사</b>(페인트·표지판, 각도 무관) /
 <b>광택</b>(차 도장, 정면에서 번쩍) / <b>거울</b>(유리·물, 거의 안 돌아오다가 정면에서만 강함)</li>
<li>같은 재질도 <b>표면 결</b>이 있습니다: 아스팔트 골재·보수 자국, 닳은 페인트, 잔디·나뭇잎의 얼룩
 (노이즈 텍스처). 레이저마다 미세한 감도 차이, 먼 거리일수록 커지는 잡음, 8비트(0~255) 양자화까지 실제 센서처럼</li>
<li>LiDAR 탑뷰 위의 <b>컬러바</b> = 실제 intensity 값(0~255)과 색의 대응</li></ul>

<h3>🤖 자율주행</h3>
<p>장애물 위치를 미리 알지 못하고 <b>LiDAR 점만 보고</b> 판단합니다.</p>
<ol>
<li><b>인지</b>: 점을 도로 기준으로 바꿔 차선별로 얼마나 비었는지 계산</li>
<li><b>판단</b>: 앞이 막히면 바로 옆 차선(내 방향 3차선 중) 중 더 많이 빈 곳으로 차선 변경 —
 옆에 차가 있으면 기다림. 중앙선 넘어 반대 차선으로는 절대 가지 않음. 모두 막히면 감속·정지</li>
<li><b>제어</b>: 초록 공(목표점)을 따라가도록 핸들을 꺾음 (Pure Pursuit)</li>
</ol>
<p>상태 표시: <b>CRUISE</b> 순항 · <b>CURVE</b> 커브 감속(곡률로 안전 속도 계산) · <b>LANE CHANGE</b> 차선 변경 · <b>BRAKING</b> 감속 ·
<b>AEB</b> 긴급제동 · <b>RED LIGHT</b> 신호 대기 · <b>STOPPED</b> 막혀서 정지</p>

<h3>💾 데이터 취득 (F9 / 툴바 ⏺ 데이터 녹화)</h3>
<p>자율주행 연구에서 쓰는 <b>KITTI / SemanticKITTI 형식</b> 그대로 저장합니다.
LiDAR가 한 바퀴 돌 때마다(10 Hz) 1프레임. 저장 위치: <i>문서\LidarDriveData\run_날짜_시간</i></p>
<table border="0" cellspacing="2" cellpadding="2">
<tr><td><b>velodyne/*.bin</b></td><td>점군 (x, y, z, 반사강도) — KITTI 형식</td></tr>
<tr><td><b>labels/*.label</b></td><td>점마다 정답 클래스 (도로·차선·인도·건물·나무·기둥·표지판·차…) + 차량 ID</td></tr>
<tr><td><b>boxes/*.txt</b></td><td>주변 차·주차차·드럼통·콘·상자의 3D 박스 정답 + 박스 안 점 개수</td></tr>
<tr><td><b>image_2/*.png</b></td><td>전방 카메라 사진</td></tr>
<tr><td><b>pcd/*.pcd</b></td><td>CloudCompare·PCL·Open3D에서 바로 열 수 있는 점군</td></tr>
<tr><td><b>poses.txt</b></td><td>매 프레임 센서 위치·방향 (KITTI odometry)</td></tr>
<tr><td><b>frames.csv</b></td><td>속도, 핸들, 가속/브레이크 입력, 자율주행 상태</td></tr>
<tr><td><b>calib.txt, meta.json</b></td><td>센서 스펙, 장착 위치, 카메라 내부/외부 파라미터</td></tr>
</table>
<p>시뮬레이터라서 <b>정답 라벨이 자동으로 100% 정확</b>하게 붙습니다 — 실제 데이터라면 사람이 일일이 표시해야 하는 부분.
메뉴 <i>데이터 취득</i>에서 PCD·카메라 저장을 켜고 끌 수 있고, 📂 버튼으로 폴더를 엽니다.
(프레임당 약 1~2 MB, 1분 녹화 ≈ 0.6~1.2 GB)</p>

<h3>💥 충돌 변형</h3>
<p>부딪혀도 바로 끝나지 않습니다. 충돌할 때마다 <b>두 물체의 무게(질량)와 다가가던 속도</b>로 운동량을 주고받고
(가벼운 쪽이 더 많이 튕겨 나감), 사라진 에너지 중 차가 흡수한 몫만큼 차체가 그 자리에서 <b>실시간으로</b> 찌그러집니다.
손상은 계속 쌓여서 HUD의 <b>DAMAGE</b>(손상률)가 <b>100%</b>가 되거나 운전석이 35 cm 넘게 밀려들면 폐차 — 그때 주행이 끝납니다.</p>
<ul>
<li><b>콘(4 kg)·드럼통(80 kg)·나무 상자(180 kg)</b>: 튕겨 날아가 굴러서 쓰러집니다. 차는 조금 느려질 뿐 계속 달림</li>
<li><b>다른 차(1.45톤)</b>: 서로 밀려나고 에너지를 반씩 나눔. 맞은 차는 비상등처럼 한동안 멈춰 섬</li>
<li><b>벽·건물·나무·교각</b>: 땅에 고정되어 있어 차가 모든 충격을 받음. <b>가드레일</b>은 비스듬히 스치면 차를 튕겨 내보냄</li>
<li><b>옆으로 박으면</b> 문·적재함(뒷부분)이 그 위치에서 찌그러지고, 중심에서 벗어난 충돌은 차를 <b>회전</b>시키고 옆으로 미끄러지게 합니다</li>
<li>같은 곳을 또 박으면 이미 찌그러진 철판은 더 단단해서 덜 들어갑니다 (x = √(x₀² + 2E/k))</li>
</ul>
<p>찌그러지는 모양: 앞부분(크럼플 존)이 <b>아코디언처럼 접히며</b> 눌리고, 옆으로 부풀고, 보닛이 위로 솟습니다.
기둥·나무에 부딪히면 U자로 파고듭니다. 순간적으로 7% 더 눌렸다가 탄성만큼 되돌아옵니다.</p>
<p>색은 실제 충돌해석 프로그램처럼 <b>변형률 등고선</b>(철판이 몇 % 늘어나거나 눌렸는지, 12단계 색 띠)입니다.
그냥 밀려나기만 한 부분은 파랗고, 접히고 구겨진 곳은 빨강·자주입니다 (왼쪽 색 막대 = 변형률 %).
충격이 크면 화면이 흔들리고 유리·도장·플라스틱 <b>파편</b>이 튀어 바닥에 흩어집니다.
충돌 후 차체를 클릭하면 그 요소의 변형량(cm)과 변형률(%)이 표시됩니다.</p>
<p>충돌하면 오른쪽에 <b>💥 충돌 분석</b> 탭이 열립니다. 전문 지식이 없어도 알 수 있게
<b>파손 등급(경미·보통·심각·전파)</b>, 무엇과 어느 방향으로 부딪혔는지,
"몇 층 높이에서 떨어진 것과 같은 충격"인지, 운전석이 몇 cm 밀려 들어왔는지,
부상 위험과 수리 수준을 정리해 줍니다. 콘처럼 가벼운 물체는 벽보다 훨씬 덜 부서집니다.</p>

<h3>🎥 쿼터니언 카메라</h3>
<p>마우스로 드래그하면 화면 위의 가상의 공(<b>아크볼</b>)을 굴리듯 카메라가 돕니다.
회전은 <b>쿼터니언</b>으로 계산해서 오일러 각처럼 특정 각도에서 축이 꼬이는 문제(짐벌 락)가 없습니다.</p>
<ul>
<li>카메라 방향 = (모드별 기준 방향) × (마우스 회전 쿼터니언)</li>
<li><b>1인칭 (운전석)</b>: 운전자 눈 위치에서 봅니다. 대시보드·A필러·실제로 돌아가는 핸들이 보이고,
 드래그하면 <b>고개를 돌려</b> 옆창 밖을 볼 수 있습니다. 차체 벽은 은면 제거(backface culling)로 안에서 투명하게 처리</li>
<li><b>3인칭</b> 모드는 기준 방향이 트럭을 따라 돌아서, 돌려 놓은 시점이 트럭 기준으로 유지됩니다</li>
<li><b>자유</b> 모드는 기준 방향이 고정이라 트럭이 돌아도 보는 방향이 그대로입니다</li>
<li><b>↺ 시점 자동 복귀</b>를 켜면 마우스를 놓고 2초 뒤 <b>Slerp</b>(구면 선형 보간)로 부드럽게 원위치</li>
</ul>

<h3>🎨 툴바 버튼</h3>
<ul>
<li><b>표면 / 와이어프레임</b>: 면으로 보기 / 선으로만 보기</li>
<li><b>메쉬 선</b>: 유한요소 메쉬 선 표시 (충돌 해석 느낌)</li>
<li><b>순항속도</b>: 자율주행 목표 속도</li>
<li><b>하늘</b>: 하늘 배경(스카이박스)</li>
<li><b>그림자</b>: 태양 방향으로 물체를 땅에 투영한 그림자</li>
<li><b>PBR 재질</b>: 물리 기반 재질 — 유리 빌딩·차 도장(클리어코트)·호수가 하늘을 반사</li>
<li><b>✨ 고품질</b>: SSAO(구석·건물 밑동의 자연스러운 그늘), FXAA(계단 현상 제거),
 원근 안개(멀수록 하늘색으로 흐려짐), 표면 질감 노이즈, 대비 보정. 느리면 끄세요</li>
<li><b>고스트</b>: 리셋 전 주행 경로를 반투명 차로 다시 재생</li>
</ul>

<h3>📦 PID 목록 (왼쪽)</h3>
<p>원본 drive-vehicle 예제처럼 부품(PID)마다 체크로 보이기/숨기기, 클릭으로 선택(노란 선)을 할 수 있습니다.
 트럭 부품은 와이어프레임 전체, 도로·건물 같은 배경은 외곽선만 표시합니다. <b>Esc</b>로 해제.</p>
)";
}

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , m_pidActorManager(new PidActorManager())
{
    m_sim.init();
    m_sim.lidar.setPipelined(true);   // LiDAR scanning overlaps with rendering

    setupUi();
    setupGuide();
    setupVtk();

    statusBar()->showMessage(ko(u8"준비 완료 — W로 출발, T로 자율주행, F1로 사용 안내"));
}

MainWindow::~MainWindow()
{
    delete m_pidActorManager;
}

// =====================================================================
// UI
// =====================================================================
void MainWindow::setupUi()
{
    setWindowTitle(ko(u8"LiDAR Drive — VTK 자율주행 & 충돌 변형 시뮬레이터"));
    resize(1680, 960);

    m_vtkWidget = new QVTKOpenGLNativeWidget(this);
    m_vtkWidget->setFocusPolicy(Qt::StrongFocus);
    m_vtkWidget->installEventFilter(this);
    setCentralWidget(m_vtkWidget);

    auto makeAction = [this](const char* text, const char* tip) {
        QAction* a = new QAction(ko(text), this);
        a->setToolTip(ko(tip));
        a->setStatusTip(ko(tip));
        return a;
    };
    auto makeToggle = [&](const char* text, const char* tip, bool on) {
        QAction* a = makeAction(text, tip);
        a->setCheckable(true);
        a->setChecked(on);
        connect(a, &QAction::toggled, this, &MainWindow::applyVisualToggles);
        return a;
    };

    QAction* surfaceAction   = makeAction(u8"표면", u8"면으로 채워서 보기");
    QAction* wireframeAction = makeAction(u8"와이어프레임", u8"메쉬 선만 보기");
    QAction* startAction     = makeAction(u8"▶ 시작", u8"시뮬레이션 시작 (Enter)");
    QAction* stopAction      = makeAction(u8"⏸ 정지", u8"일시정지 (Enter)");
    QAction* resetAction     = makeAction(u8"⟲ 리셋", u8"처음 위치로 (R). 방금 주행은 고스트로 저장");

    connect(surfaceAction, &QAction::triggered, this, &MainWindow::setSurfaceMode);
    connect(wireframeAction, &QAction::triggered, this, &MainWindow::setWireframeMode);
    connect(startAction, &QAction::triggered, this, &MainWindow::startMotion);
    connect(stopAction, &QAction::triggered, this, &MainWindow::stopMotion);
    connect(resetAction, &QAction::triggered, this, &MainWindow::resetMotion);

    m_autopilotAction = makeAction(u8"🤖 자율주행",
        u8"자율주행 켜기/끄기 (T): LiDAR 점만 보고 차선 변경·감속·긴급제동");
    m_autopilotAction->setCheckable(true);
    connect(m_autopilotAction, &QAction::toggled, this, &MainWindow::setAutopilot);

    m_lidarAction  = makeToggle(u8"LiDAR 점", u8"LiDAR가 감지한 점 표시 (L). 색 = 반사 강도 (파랑 약함 → 빨강 강함)", true);
    m_skyAction    = makeToggle(u8"하늘", u8"하늘 배경 (스카이박스)", true);
    m_shadowAction = makeToggle(u8"그림자", u8"태양 방향 평면 투영 그림자", true);
    m_pbrAction    = makeToggle(u8"PBR 재질", u8"물리 기반 재질: 유리 빌딩·차 도장·호수가 하늘을 반사", true);
    m_hqAction     = makeToggle(u8"✨ 고품질",
        u8"고품질 그래픽: SSAO(구석 그늘), FXAA(계단 현상 제거), 원근 안개, 표면 질감. 느리면 끄세요", true);
    m_ghostAction  = makeToggle(u8"고스트", u8"이전 주행을 반투명 차로 재생 (G)", true);
    m_edgesAction  = makeToggle(u8"메쉬 선", u8"유한요소 메쉬 선 표시 (충돌 해석 느낌)", false);
    m_textureAction = makeToggle(u8"텍스처", u8"텍스처 매핑: 아스팔트·잔디·콘크리트 (절차적으로 만든 이미지, 밉맵 + 비등방성 필터링)", true);
    m_recordAction = makeAction(u8"⏺ 데이터 녹화",
        u8"데이터 취득 시작/정지 (F9): LiDAR 한 바퀴(10 Hz)마다 점군·라벨·3D 박스·카메라 이미지·포즈를 저장 "
        u8"(KITTI / SemanticKITTI 형식)");
    m_recordAction->setCheckable(true);
    connect(m_recordAction, &QAction::toggled, this, &MainWindow::setRecording);
    m_recPcdAction = makeAction(u8"PCD 파일도 저장 (CloudCompare·PCL용)",
        u8"pcd/*.pcd 추가 저장 (프레임당 약 1 MB 더 사용)");
    m_recPcdAction->setCheckable(true);
    m_recPcdAction->setChecked(true);
    m_recCamAction = makeAction(u8"전방 카메라 이미지 저장", u8"image_2/*.png (오른쪽 위 카메라 화면)");
    m_recCamAction->setCheckable(true);
    m_recCamAction->setChecked(true);
    m_openDataAction = makeAction(u8"📂 데이터 폴더", u8"녹화된 데이터가 있는 폴더 열기");
    connect(m_openDataAction, &QAction::triggered, this, [this] {
        const QString dir = m_lastDataDir.isEmpty()
            ? QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation) + "/LidarDriveData"
            : m_lastDataDir;
        QDir().mkpath(dir);
        QDesktopServices::openUrl(QUrl::fromLocalFile(dir));
    });
    m_helpAction   = makeAction(u8"❓ 도움말", u8"사용 안내 열기/닫기 (F1)");
    m_helpAction->setCheckable(true);
    m_helpAction->setChecked(true);

    // menus
    QMenu* fileMenu = menuBar()->addMenu(ko(u8"파일"));
    QAction* exitAction = new QAction(ko(u8"종료"), this);
    connect(exitAction, &QAction::triggered, this, &QMainWindow::close);
    fileMenu->addAction(exitAction);

    QMenu* simMenu = menuBar()->addMenu(ko(u8"시뮬레이션"));
    for (QAction* a : { startAction, stopAction, resetAction, m_autopilotAction })
        simMenu->addAction(a);

    QMenu* viewMenu = menuBar()->addMenu(ko(u8"보기"));
    for (QAction* a : { surfaceAction, wireframeAction, m_edgesAction })
        viewMenu->addAction(a);
    viewMenu->addSeparator();
    for (QAction* a : { m_lidarAction, m_skyAction, m_shadowAction, m_pbrAction, m_textureAction, m_hqAction, m_ghostAction })
        viewMenu->addAction(a);

    QMenu* dataMenu = menuBar()->addMenu(ko(u8"데이터 취득"));
    dataMenu->addAction(m_recordAction);
    dataMenu->addSeparator();
    dataMenu->addAction(m_recPcdAction);
    dataMenu->addAction(m_recCamAction);
    dataMenu->addSeparator();
    dataMenu->addAction(m_openDataAction);

    QMenu* helpMenu = menuBar()->addMenu(ko(u8"도움말"));
    helpMenu->addAction(m_helpAction);

    // toolbar 1: simulation
    QToolBar* simBar = addToolBar(ko(u8"시뮬레이션"));
    simBar->setToolButtonStyle(Qt::ToolButtonTextOnly);
    simBar->addAction(startAction);
    simBar->addAction(stopAction);
    simBar->addAction(resetAction);

    auto* startLabel = new QLabel(ko(u8"  출발 "), this);
    startLabel->setToolTip(ko(u8"리셋(R)할 때 차가 출발하는 곳"));
    simBar->addWidget(startLabel);
    m_startCombo = new QComboBox(this);
    m_startCombo->addItems({ ko(u8"도심"), ko(u8"장애물 코스 (숲길)"), ko(u8"고속도로"), ko(u8"교외 주택가"),
                             ko(u8"도로망 · 고가도로 앞"), ko(u8"도로망 · 지하차도 앞") });
    m_startCombo->setFocusPolicy(Qt::NoFocus);
    m_startCombo->setToolTip(ko(u8"출발 위치: 도심(교차로·양방향 교통) / 장애물 코스 / 고속도로 / 교외 주택가"));
    connect(m_startCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &MainWindow::setStartPlace);
    simBar->addWidget(m_startCombo);

    auto* vehicleLabel = new QLabel(ko(u8"  🚗 차종 (1~4) "), this);
    vehicleLabel->setToolTip(ko(u8"운전할 차 (모양·무게·출력·센서 위치가 바뀝니다)"));
    simBar->addWidget(vehicleLabel);
    m_vehicleCombo = new QComboBox(this);
    for (int t = 0; t < VehicleShapes::TypeCount; ++t)
        m_vehicleCombo->addItem(ko(VehicleShapes::spec(t).nameKo));
    m_vehicleCombo->setFocusPolicy(Qt::NoFocus);
    m_vehicleCombo->setToolTip(ko(u8"트럭 3.5톤 190 kW / 세단 1.55톤 / SUV 2.2톤 / 스포츠카 1.45톤 330 kW\n"
                                  u8"모두 원본 예제와 같은 차량 좌표계(PID 1~4)라 충돌 변형·LiDAR가 그대로 동작"));
    connect(m_vehicleCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &MainWindow::setVehicleType);
    simBar->addWidget(m_vehicleCombo);
    {
        // the same choice in a menu at the very top (the toolbar can be cut off in a small window)
        QMenu* menu = menuBar()->addMenu(ko(u8"🚗 차종 바꾸기"));
        auto* group = new QActionGroup(this);
        for (int t = 0; t < VehicleShapes::TypeCount; ++t)
        {
            const auto& sp = VehicleShapes::spec(t);
            QAction* a = menu->addAction(QString(ko(u8"%1  —  %2톤, %3 kW")).arg(ko(sp.nameKo))
                                             .arg(sp.mass / 1000.0, 0, 'f', 2).arg(sp.power / 1000.0, 0, 'f', 0));
            a->setCheckable(true);
            a->setChecked(t == 0);
            a->setShortcut(QKeySequence(Qt::Key_1 + t));
            a->setShortcutContext(Qt::ApplicationShortcut);
            group->addAction(a);
            connect(a, &QAction::triggered, this, [this, t]() { m_vehicleCombo->setCurrentIndex(t); });
            m_vehicleActions.push_back(a);
        }
        connect(m_vehicleCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int t) {
            if (t >= 0 && t < static_cast<int>(m_vehicleActions.size()))
                m_vehicleActions[t]->setChecked(true);
        });
    }

    simBar->addSeparator();
    simBar->addAction(m_autopilotAction);

    auto* cruiseLabel = new QLabel(ko(u8"  순항속도 "), this);
    cruiseLabel->setToolTip(ko(u8"자율주행 목표 속도"));
    simBar->addWidget(cruiseLabel);
    m_cruiseSpin = new QDoubleSpinBox(this);
    m_cruiseSpin->setRange(10.0, 120.0);
    m_cruiseSpin->setSingleStep(5.0);
    m_cruiseSpin->setValue(43.0);
    m_cruiseSpin->setSuffix(" km/h");
    m_cruiseSpin->setDecimals(0);
    m_cruiseSpin->setFocusPolicy(Qt::ClickFocus);
    m_cruiseSpin->setToolTip(ko(u8"자율주행 목표 속도"));
    simBar->addWidget(m_cruiseSpin);

    simBar->addSeparator();
    auto* camLabel = new QLabel(ko(u8" 카메라 "), this);
    camLabel->setToolTip(ko(u8"시점 바꾸기 (C)"));
    simBar->addWidget(camLabel);
    m_cameraCombo = new QComboBox(this);
    m_cameraCombo->addItems({ ko(u8"3인칭 (뒤에서)"), ko(u8"위에서"), ko(u8"자유 (방향 고정)"), ko(u8"시네마틱"),
                              ko(u8"1인칭 (운전석)") });
    m_cameraCombo->setFocusPolicy(Qt::NoFocus);
    m_cameraCombo->setToolTip(ko(u8"V = 1인칭 ↔ 3인칭 전환 (C = 모든 시점 순서대로)\n"
                                 u8"1인칭: 운전석에 앉은 시점, 드래그로 고개 돌리기 / 3인칭: 차 뒤를 따라감 / "
                                 u8"위에서: 하늘에서 내려다봄 / 자유: 보는 방향 고정 / 시네마틱: 차 주위를 돌며 촬영\n"
                                 u8"마우스 드래그 = 쿼터니언 회전, 휠 = 확대/축소, 더블클릭 또는 Home = 원위치"));
    connect(m_cameraCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &MainWindow::setCameraMode);
    simBar->addWidget(m_cameraCombo);

    m_autoReturnAction = makeAction(u8"↺ 시점 자동 복귀",
        u8"마우스를 놓고 2초 뒤 카메라가 Slerp(구면 선형 보간)로 부드럽게 원래 시점으로 돌아옴");
    m_autoReturnAction->setCheckable(true);
    m_autoReturnAction->setChecked(true);
    simBar->addAction(m_autoReturnAction);
    simBar->addSeparator();
    simBar->addAction(m_helpAction);

    // toolbar 2: display
    QToolBar* viewBar = addToolBar(ko(u8"표시"));
    viewBar->setToolButtonStyle(Qt::ToolButtonTextOnly);
    viewBar->addAction(surfaceAction);
    viewBar->addAction(wireframeAction);
    viewBar->addAction(m_edgesAction);
    viewBar->addSeparator();
    for (QAction* a : { m_lidarAction, m_skyAction, m_shadowAction, m_pbrAction, m_textureAction, m_hqAction, m_ghostAction })
        viewBar->addAction(a);

    // toolbar 3: data acquisition
    QToolBar* dataBar = addToolBar(ko(u8"데이터 취득"));
    dataBar->setToolButtonStyle(Qt::ToolButtonTextOnly);
    dataBar->addAction(m_recordAction);
    dataBar->addAction(m_openDataAction);

    // PID dock (original example)
    QDockWidget* modelDock = new QDockWidget(ko(u8"PID / 부품"), this);
    m_treeWidget = new QTreeWidget(modelDock);
    m_treeWidget->setHeaderLabel(ko(u8"체크 = 보이기, 클릭 = 선택"));
    m_treeWidget->installEventFilter(this);
    // no keyboard focus at start-up: Qt would make the first row "current" -> the truck got selected
    m_treeWidget->setFocusPolicy(Qt::ClickFocus);
    modelDock->setWidget(m_treeWidget);
    addDockWidget(Qt::LeftDockWidgetArea, modelDock);

    connect(m_treeWidget, &QTreeWidget::itemChanged, this, &MainWindow::onTreeItemChanged);
    connect(m_treeWidget, &QTreeWidget::currentItemChanged, this, &MainWindow::onCurrentTreeItemChanged);

    // status bar: section + what the autopilot is doing (Korean)
    m_sectionLabel = new QLabel(this);
    m_stateLabel = new QLabel(this);
    m_stateLabel->setMinimumWidth(520);
    statusBar()->addPermanentWidget(m_sectionLabel);
    statusBar()->addPermanentWidget(m_stateLabel);
}

void MainWindow::setupGuide()
{
    m_guideDock = new QDockWidget(ko(u8"사용 안내 (F1)"), this);
    auto* browser = new QTextBrowser(m_guideDock);
    browser->setHtml(ko(kGuideHtml));
    browser->setOpenExternalLinks(false);
    browser->setFocusPolicy(Qt::NoFocus);
    m_guideDock->setWidget(browser);
    m_guideDock->setMinimumWidth(330);
    addDockWidget(Qt::RightDockWidgetArea, m_guideDock);

    connect(m_helpAction, &QAction::toggled, m_guideDock, &QDockWidget::setVisible);
    connect(m_guideDock, &QDockWidget::visibilityChanged, m_helpAction, &QAction::setChecked);

    // crash analysis panel (shown as a tab next to the guide when a crash happens)
    m_crashDock = new QDockWidget(ko(u8"💥 충돌 분석"), this);
    m_crashBrowser = new QTextBrowser(m_crashDock);
    m_crashBrowser->setFocusPolicy(Qt::NoFocus);
    clearCrashReport();
    m_crashDock->setWidget(m_crashBrowser);
    addDockWidget(Qt::RightDockWidgetArea, m_crashDock);
    tabifyDockWidget(m_guideDock, m_crashDock);
    m_guideDock->raise();
}

void MainWindow::clearCrashReport()
{
    m_crashBrowser->setHtml(ko(u8"<h2>💥 충돌 분석</h2><p>아직 충돌이 없습니다. 무언가에 부딪히면 "
                               u8"얼마나 부서졌는지 여기에 쉽게 정리해서 보여줍니다.</p>"));
}

void MainWindow::showCrashReport()
{
    const DriveSim::CrashReport& r = m_sim.report;

    static const char* colors[] = { "#3fb950", "#d4b106", "#f0883e", "#f85149" };
    static const char* names[] = { u8"경미", u8"보통", u8"심각", u8"전파" };
    QString gauge = "<table width='100%' cellspacing='3' cellpadding='6'><tr>";
    for (int g = 1; g <= 4; ++g)
    {
        const bool on = (g == r.grade);
        gauge += QString("<td align='center' bgcolor='%1' style='color:%2;%3'>%4</td>")
                     .arg(on ? colors[g - 1] : "#30363d")
                     .arg(on ? "#000" : "#8b949e")
                     .arg(on ? "font-weight:bold;font-size:15px;" : "")
                     .arg(ko(names[g - 1]));
    }
    gauge += "</tr></table>";

    auto bar = [](double pct, const char* color) {
        const int w = std::clamp(static_cast<int>(pct), 1, 100);
        return QString("<table width='100%' cellspacing='0' cellpadding='0'><tr>"
                       "<td width='%1%' bgcolor='%2' height='10'></td><td bgcolor='#30363d'></td></tr></table>")
            .arg(w).arg(color);
    };

    const double floors = r.dropHeightM / 3.0;
    QString h;
    h += ko(u8"<h2>💥 충돌 분석</h2>");
    {
        // live line: refreshed twice a second while the panel is open
        const auto& im = m_sim.lastImpact;
        const double ago = m_sim.time - im.time;
        h += QString(ko(u8"<p style='background:#21262d;padding:4px'>%1 <b>실시간</b> — 현재 속도 <b>%2 km/h</b> · "
                        u8"손상률 <b>%3%</b> · 마지막 충돌 %4초 전 · 변형 진행 %5</p>"))
                 .arg(m_sim.crash.happened ? ko(u8"⛔") : ko(u8"🔴"))
                 .arg(std::abs(m_sim.vehicle.speed()) * 3.6, 0, 'f', 0)
                 .arg(m_sim.damagePct, 0, 'f', 0)
                 .arg(ago, 0, 'f', 1)
                 .arg(m_sim.deformer.animating() ? ko(u8"<b>찌그러지는 중</b>") : ko(u8"끝남"));
    }
    h += QString(ko(u8"<p style='font-size:16px'><b>파손 등급: <span style='color:%1'>%2</span> (%3/4)</b></p>"))
             .arg(colors[r.grade - 1]).arg(ko(r.gradeKo)).arg(r.grade);
    h += gauge;
    h += QString(ko(u8"<p style='font-size:14px'>차체 손상률 <b>%1%</b> — 충돌 %2회 누적 "
                    u8"(<b>100%</b>가 되거나 운전석이 35 cm 넘게 밀려들면 폐차, 주행 종료)</p>"))
             .arg(std::min(r.damagePct, 999.0), 0, 'f', 0).arg(r.impacts);
    h += bar(std::min(100.0, r.damagePct), colors[r.grade - 1]);
    h += QString(ko(u8"<p>마지막 충돌: 시속 <b>%1 km</b>로 <b>%2</b>(%4)을(를) <b>%3</b>으로 들이받았습니다.</p>"))
             .arg(r.speedKmh, 0, 'f', 0).arg(ko(r.objectKo)).arg(ko(r.sideKo))
             .arg(r.objectMassKg > 0.0 ? QString(ko(u8"%1 kg, 밀려남")).arg(r.objectMassKg, 0, 'f', 0)
                                       : ko(u8"땅에 고정됨"));

    h += ko(u8"<h3>얼마나 셌나?</h3><ul>");
    h += QString(ko(u8"<li>두 물체가 서로 다가가던 속도 <b>%1 km/h</b>. 무게가 가벼운 쪽이 더 많이 튕겨 나가고 "
                    u8"(운동량 보존), 사라진 에너지 중 <b>%2%</b>를 이 차가 찌그러지며 흡수<br>"
                    u8"→ 단단한 벽에 <b>%3 km/h</b>로 부딪힌 것과 같은 손상</li>"))
             .arg(r.relSpeedKmh, 0, 'f', 0).arg(r.severity * 100.0, 0, 'f', 0).arg(r.effectiveKmh, 0, 'f', 0);
    h += QString(ko(u8"<li>몸이 받는 충격: 최대 <b>%1 g</b> (평균 %2 g), 부딪힌 뒤 <b>%3 ms</b> 만에 최대로 찌그러짐<br>"
                    u8"→ 비교: 롤러코스터 4~5 g · 전투기 조종사 9 g · 충돌시험(시속 56 km 정면) 약 40 g</li>"))
             .arg(r.peakG, 0, 'f', 0).arg(r.avgG, 0, 'f', 0).arg(r.pulseMs, 0, 'f', 0);
    h += QString(ko(u8"<li>충돌로 사라진 에너지 <b>%1 kJ</b>, 그중 차체가 흡수 <b>%4 kJ</b><br>"
                    u8"→ <b>%2 m</b> 높이, 약 <b>%3층</b> 건물에서 떨어진 것과 같은 충격</li>"))
             .arg(r.energyKJ, 0, 'f', 0).arg(r.dropHeightM, 0, 'f', 1).arg(std::max(0.0, floors), 0, 'f', 0)
             .arg(r.carKJ, 0, 'f', 0);
    h += ko(u8"</ul>");

    h += ko(u8"<h3>얼마나 부서졌나?</h3><ul>");
    h += QString(ko(u8"<li>가장 많이 찌그러진 곳: <b>%1 cm</b> 밀려 들어감</li>")).arg(r.maxCrushM * 100.0, 0, 'f', 0);
    h += QString(ko(u8"<li>운전석 공간 침범: <b>%1 cm</b> %2</li>"))
             .arg(r.intrusionM * 100.0, 0, 'f', 0)
             .arg(r.intrusionM > 0.3 ? ko(u8"⚠️ 운전자 공간이 크게 줄어듦") : r.intrusionM > 0.05 ? ko(u8"(조금)") : ko(u8"(없음)"));
    h += QString(ko(u8"<li>가장 심하게 늘어나거나 눌린 철판: 변형률 <b>%1%</b> "
                    u8"(차체 강판은 보통 20~40%를 넘으면 찢어지거나 접힘)</li>")).arg(r.maxStrainPct, 0, 'f', 0);
    h += QString(ko(u8"<li>차체 손상 범위: <b>%1%</b> (차체를 이루는 작은 요소 중 3 cm 이상 찌그러진 비율)</li>"))
             .arg(r.damagedPct, 0, 'f', 0);
    h += bar(r.damagedPct, colors[r.grade - 1]);
    h += ko(u8"</ul>");

    // wheels / axles
    {
        h += ko(u8"<h3>바퀴 · 차축</h3><ul>");
        bool any = false;
        const size_t nAx = m_sim.wheels.size() / 2;
        for (size_t k = 0; k < m_sim.wheels.size(); ++k)
        {
            const auto& w = m_sim.wheels[k];
            if (w.damage < 0.05)
                continue;
            any = true;
            const size_t ax = k / 2;
            const char* where = ax == 0 ? u8"앞" : (ax + 1 == nAx ? u8"뒤" : u8"가운데");
            QString state;
            if (w.detached)
                state = ko(u8"<span style='color:#f85149'><b>바퀴 이탈</b></span> — 허브가 바닥에 끌림 (차체가 그쪽으로 주저앉음)");
            else if (w.damage > 0.6)
                state = QString(ko(u8"<span style='color:#f0883e'><b>차축·서스펜션 휨</b></span> — 캠버 %1°, 토 %2°"))
                            .arg(std::abs(w.camber) * 57.3, 0, 'f', 0).arg(std::abs(w.toe) * 57.3, 0, 'f', 0);
            else
                state = QString(ko(u8"<span style='color:#d4b106'>휠 얼라인먼트 틀어짐</span> — 캠버 %1°, 토 %2°"))
                            .arg(std::abs(w.camber) * 57.3, 0, 'f', 0).arg(std::abs(w.toe) * 57.3, 0, 'f', 0);
            h += QString("<li>%1 %2: %3 (%4%)</li>")
                     .arg(ko(where)).arg(ko(w.left() ? u8"왼쪽" : u8"오른쪽")).arg(state)
                     .arg(std::min(100.0, w.damage / 1.3 * 100.0), 0, 'f', 0);
        }
        if (!any)
            h += ko(u8"<li>모두 정상</li>");
        else
        {
            const auto& v = m_sim.vehicle;
            h += QString(ko(u8"<li>주행에 미치는 영향: 접지력 앞 <b>%1%</b> · 뒤 <b>%2%</b>, 핸들이 <b>%3쪽으로 %4°</b> 쏠림, "
                            u8"구름 저항 +%5 kN</li>"))
                     .arg(v.wheelGrip[0] * 100.0, 0, 'f', 0).arg(v.wheelGrip[1] * 100.0, 0, 'f', 0)
                     .arg(ko(v.wheelPull >= 0.0 ? u8"왼" : u8"오른")).arg(std::abs(v.wheelPull) * 57.3, 0, 'f', 1)
                     .arg(v.wheelDrag / 1000.0, 0, 'f', 1);
        }
        h += ko(u8"</ul>");
    }

    // every impact of this run
    {
        h += ko(u8"<h3>충돌 기록</h3><table width='100%' cellspacing='0' cellpadding='3'>"
                u8"<tr style='color:#8b949e'><td>시각</td><td>물체</td><td>무게</td><td>속도</td><td>충격</td>"
                u8"<td>손상률</td></tr>");
        const auto& log = m_sim.impactLog;
        const size_t first = log.size() > 12 ? log.size() - 12 : 0;
        for (size_t k = log.size(); k-- > first;)
        {
            const auto& e = log[k];
            const double add = e.damageAfter - e.damageBefore;
            const char* col = add < 2.0 ? "#3fb950" : add < 15.0 ? "#d4b106" : add < 40.0 ? "#f0883e" : "#f85149";
            h += QString("<tr><td>%1 s</td><td>%2%3</td><td>%4</td><td>%5 km/h</td><td>%6 g</td>"
                         "<td><span style='color:%7'>+%8</span> → %9%</td></tr>")
                     .arg(e.time, 0, 'f', 1)
                     .arg(ko(World::kindNameKo(e.kind)))
                     .arg(e.knocked ? ko(u8" 🪃") : QString())
                     .arg(e.objectMassKg > 0.0 ? QString("%1 kg").arg(e.objectMassKg, 0, 'f', 0) : ko(u8"고정"))
                     .arg(e.speedKmh, 0, 'f', 0)
                     .arg(e.peakG, 0, 'f', 0)
                     .arg(col)
                     .arg(add, 0, 'f', 1)
                     .arg(e.damageAfter, 0, 'f', 0);
        }
        h += ko(u8"</table><p style='color:#888'>🪃 = 가벼워서 튕겨 날아감 · 고정 = 땅에 박힌 물체(차가 충격을 다 받음)</p>");
    }

    h += ko(u8"<h3>결과</h3><ul>");
    h += QString(ko(u8"<li>🧑 운전자 부상 위험: <b>%1</b></li>")).arg(ko(r.injuryKo));
    h += QString(ko(u8"<li>🔧 수리: %1</li>")).arg(ko(r.repairKo));
    if (!r.wrecked)
        h += ko(u8"<li>🚗 아직 달릴 수 있습니다. 더 부딪히면 손상이 쌓입니다 (<b>R</b>: 새 차로 다시 시작)</li>");
    h += ko(u8"</ul>");

    h += ko(u8"<h3>화면 색 읽는 법 (변형률 등고선)</h3>"
            u8"<p>실제 충돌해석 프로그램처럼 철판이 <b>얼마나 늘어나거나 눌렸는지(변형률 %)</b>를 12단계 색 띠로 칠합니다. "
            u8"그냥 밀려나기만 한 부분은 파랗고, 접히고 구겨진 곳일수록 빨갛습니다.</p>"
            u8"<p><span style='background:#0000b3;color:white'>&nbsp;남색 = 멀쩡&nbsp;</span> → "
            u8"<span style='background:#00d9e6'>&nbsp;하늘&nbsp;</span> → "
            u8"<span style='background:#4de600'>&nbsp;초록&nbsp;</span> → "
            u8"<span style='background:#ffe600'>&nbsp;노랑&nbsp;</span> → "
            u8"<span style='background:#f21400;color:white'>&nbsp;빨강&nbsp;</span> → "
            u8"<span style='background:#d90073;color:white'>&nbsp;자주 = 찢어지거나 완전히 접힘&nbsp;</span></p>"
            u8"<p>앞부분이 <b>아코디언처럼 접히며</b> 충격을 흡수하고(크럼플 존), 보닛은 위로 솟고, 기둥에 부딪히면 "
            u8"U자로 파고듭니다. 충돌 순간에는 6% 더 눌렸다가 탄성만큼 조금 되돌아옵니다.</p>"
            u8"<p>차체를 클릭하면 그 부분이 몇 cm 찌그러졌는지와 변형률이 아래 상태 표시줄에 나옵니다.<br>"
            u8"<b>R</b>을 누르면 다시 시작합니다.</p>"
            u8"<p style='color:#888'>※ 실제 유한요소 충돌해석이 아니라, 충돌 에너지(½mv² = ½kx²)·물체 단단함·충돌 방향으로 "
            u8"찌그러짐을 근사한 교육용 모델입니다. 변형률은 각 육면체 요소의 모서리 길이 변화로 계산합니다.</p>");

    const int scroll = m_crashBrowser->verticalScrollBar()->value();
    m_crashBrowser->setHtml(h);
    m_crashBrowser->verticalScrollBar()->setValue(scroll);
    m_reportClock.restart();
}

// =====================================================================
// VTK scene
// =====================================================================
void MainWindow::setupPalette()
{
    m_paletteLut = vtkSmartPointer<vtkLookupTable>::New();
    m_paletteLut->SetNumberOfTableValues(Palette::Count);
    m_paletteLut->SetTableRange(-0.5, Palette::Count - 0.5);
    m_paletteLut->Build();
    for (int i = 0; i < Palette::Count; ++i)
    {
        const double* c = Palette::color(i);
        m_paletteLut->SetTableValue(i, c[0], c[1], c[2], 1.0);
    }
    m_pidActorManager->setLookupTable(m_paletteLut, -0.5, Palette::Count - 0.5);
}

void MainWindow::setupVtk()
{
    m_renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    m_renderer = vtkSmartPointer<vtkRenderer>::New();

    m_vtkWidget->setRenderWindow(m_renderWindow);
    m_renderWindow->SetNumberOfLayers(3);
    m_renderWindow->AddRenderer(m_renderer);

    // vehicle pose -> one transform shared by every vehicle actor
    m_vehicleTransform = vtkSmartPointer<vtkTransform>::New();
    m_sim.vehicle.toTransform(m_vehicleTransform);

    setupPalette();
    m_pidActorManager->buildFromPidGrids(m_renderer, m_sim.world.pidGrids());
    for (int pid : m_pidActorManager->getPidList())
        if (World::isVehiclePid(pid))
            m_pidActorManager->setUserTransform(pid, m_vehicleTransform);
    if (vtkActor* t = m_pidActorManager->getActorByPid(World::PidTunnel))
        t->GetProperty()->SetDiffuse(0.55);

    m_renderer->SetBackground(0.15, 0.18, 0.22);
    m_renderer->GetActiveCamera()->SetViewAngle(45.0);

    // 3D overlay on top of the main scene: shares camera and depth buffer
    m_overlayRenderer = vtkSmartPointer<vtkRenderer>::New();
    m_overlayRenderer->SetLayer(1);
    m_overlayRenderer->PreserveColorBufferOn();
    m_overlayRenderer->PreserveDepthBufferOn();
    m_overlayRenderer->InteractiveOff();
    m_overlayRenderer->SetActiveCamera(m_renderer->GetActiveCamera());
    m_renderWindow->AddRenderer(m_overlayRenderer);

    // PID tree
    m_treeWidget->blockSignals(true);
    for (int pid : m_pidActorManager->getPidList())
    {
        auto* item = new QTreeWidgetItem(m_treeWidget);
        item->setText(0, QString("PID %1  ").arg(pid) + ko(World::pidNameKo(pid)));
        item->setToolTip(0, QString("PID %1: %2").arg(pid).arg(World::pidName(pid)));
        item->setCheckState(0, Qt::Checked);
        item->setData(0, Qt::UserRole, pid);
    }
    m_treeWidget->expandAll();
    m_treeWidget->blockSignals(false);

    // picking (original example). Mouse input is handled in eventFilter():
    // drag = quaternion arcball camera, click without dragging = pick a cell.
    m_cellPicker = vtkSmartPointer<vtkCellPicker>::New();
    m_cellPicker->SetTolerance(0.0005);

    // the camera is fully controlled by our quaternion code -> VTK style that does nothing
    vtkNew<vtkInteractorStyleUser> noCameraStyle;
    m_renderWindow->GetInteractor()->SetInteractorStyle(noCameraStyle);

    // animation timer
    m_motionTimer = new QTimer(this);
    m_motionTimer->setTimerType(Qt::PreciseTimer);
    connect(m_motionTimer, &QTimer::timeout, this, &MainWindow::updateMotion);

    setupTraffic();
    setupEnvironment();
    setupLidarVisuals();
    setupGhost();
    setupCockpit();
    setupHud();
    setupFrontCamera();
    setupSignals();
    setupDebris();
    setupGauges();
    setupLidarView();
    setupMinimap();

    setupTextures();
    applyVisualToggles();
    refreshAll();
    statusBar()->showMessage(ko(u8"차종은 숫자키 1~4 (1 트럭 · 2 세단 · 3 SUV · 4 스포츠카) 또는 맨 위 메뉴 '🚗 차종 바꾸기'로 "
                                u8"바꿀 수 있습니다"), 15000);
}

void MainWindow::setupTraffic()
{
    for (size_t i = 0; i < m_sim.npcs.size(); ++i)
    {
        auto t = vtkSmartPointer<vtkTransform>::New();
        m_sim.npcs[i].toTransform(t);
        m_npcTransforms.push_back(t);

        vtkNew<vtkDataSetMapper> mapper;
        mapper->SetInputData(m_sim.world.npcGrid(i));
        mapper->SetScalarModeToUseCellData();
        mapper->SetLookupTable(m_paletteLut);
        mapper->SetScalarRange(-0.5, Palette::Count - 0.5);

        auto actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->SetUserTransform(t);
        actor->GetProperty()->SetEdgeColor(0.08, 0.08, 0.08);
        m_renderer->AddActor(actor);
        m_npcActors.push_back(actor);
    }
}

void MainWindow::setupEnvironment()
{
    // ---- terrain: hills, park, lake shore (vertex colors from World) ----
    vtkNew<vtkPolyDataMapper> terrainMapper;
    terrainMapper->SetInputData(m_sim.world.terrain());
    terrainMapper->SetScalarModeToUsePointData();
    terrainMapper->SetColorModeToDirectScalars();
    m_terrainActor = vtkSmartPointer<vtkActor>::New();
    m_terrainActor->SetMapper(terrainMapper);
    m_terrainActor->GetProperty()->SetSpecular(0.0);
    m_terrainActor->GetProperty()->SetAmbient(0.15);
    m_terrainActor->PickableOff();
    m_renderer->AddActor(m_terrainActor);

    // ---- lake: dark, glossy -> reflects the sky (PBR + image based lighting) ----
    vtkNew<vtkPolyDataMapper> waterMapper;
    waterMapper->SetInputData(m_sim.world.water());
    m_waterActor = vtkSmartPointer<vtkActor>::New();
    m_waterActor->SetMapper(waterMapper);
    m_waterActor->GetProperty()->SetColor(0.06, 0.16, 0.20);
    m_waterActor->GetProperty()->SetSpecular(0.9);
    m_waterActor->GetProperty()->SetSpecularPower(80.0);
    m_waterActor->PickableOff();
    m_renderer->AddActor(m_waterActor);

    // ---- procedural sky with clouds (equirectangular texture -> vtkSkybox + IBL) ----
    const int W = 2048, H = 1024;
    vtkNew<vtkImageData> img;
    img->SetDimensions(W, H, 1);
    img->AllocateScalars(VTK_UNSIGNED_CHAR, 3);
    const double sunAz = 20.0 * kPi / 180.0, sunEl = 47.0 * kPi / 180.0;
    auto hash = [](int x, int y) {
        uint32_t h = static_cast<uint32_t>(x) * 374761393u + static_cast<uint32_t>(y) * 668265263u;
        h = (h ^ (h >> 13)) * 1274126177u;
        return (h ^ (h >> 16)) / 4294967295.0;
    };
    auto noise = [&](double x, double y) {
        const int ix = static_cast<int>(std::floor(x)), iy = static_cast<int>(std::floor(y));
        double fx = x - ix, fy = y - iy;
        fx = fx * fx * (3 - 2 * fx);
        fy = fy * fy * (3 - 2 * fy);
        const double a = hash(ix, iy), b = hash(ix + 1, iy), c = hash(ix, iy + 1), d = hash(ix + 1, iy + 1);
        const double ab = a + (b - a) * fx, cd = c + (d - c) * fx;
        return ab + (cd - ab) * fy;
    };
    auto fbm = [&](double x, double y) {
        double sum = 0, amp = 0.5, norm = 0;
        for (int o = 0; o < 6; ++o)
        {
            sum += amp * noise(x, y);
            norm += amp;
            x = x * 2.1 + 5.3;
            y = y * 2.1 - 1.7;
            amp *= 0.5;
        }
        return sum / norm;
    };
    const double haze[3] = { 0.80, 0.84, 0.88 };
    for (int j = 0; j < H; ++j)
    {
        const double el = ((j + 0.5) / H - 0.5) * kPi;
        for (int i = 0; i < W; ++i)
        {
            const double az = (i + 0.5) / W * 2.0 * kPi;
            double c[3];
            if (el >= 0.0)
            {
                const double t = std::pow(std::sin(el), 0.5);
                const double zenith[3] = { 0.17, 0.37, 0.72 };
                for (int k = 0; k < 3; ++k) c[k] = haze[k] * (1 - t) + zenith[k] * t;
                // clouds on a plane above the camera: (dir.xy / dir.z) -> no stretching at the zenith
                const double dz = std::sin(el) + 0.08;
                const double px = std::cos(el) * std::cos(az) / dz, py = std::cos(el) * std::sin(az) / dz;
                const double cov = fbm(px * 1.6 + 3.0, py * 1.6 - 2.0);
                double cloud = std::clamp((cov - 0.52) / 0.22, 0.0, 1.0);
                cloud *= std::clamp(std::sin(el) * 6.0, 0.0, 1.0);          // fade into the horizon haze
                const double shade = 0.78 + 0.22 * fbm(px * 4.0 - 7.0, py * 4.0 + 1.0);
                for (int k = 0; k < 3; ++k) c[k] = c[k] * (1 - cloud) + 0.97 * shade * cloud;
            }
            else
            {
                const double t = std::min(1.0, -el * 5.0);
                const double ground[3] = { 0.30, 0.36, 0.28 };
                for (int k = 0; k < 3; ++k) c[k] = haze[k] * (1 - t) + ground[k] * t;
            }
            const double cosAng = std::sin(el) * std::sin(sunEl) +
                                  std::cos(el) * std::cos(sunEl) * std::cos(az - sunAz);
            const double ang = std::acos(std::clamp(cosAng, -1.0, 1.0));
            const double glow = std::exp(-ang * ang / 0.0015) + 0.22 * std::exp(-ang * ang / 0.05);
            for (int k = 0; k < 3; ++k) c[k] = std::min(1.0, c[k] + glow * (k == 2 ? 0.8 : 1.0));

            auto* px = static_cast<unsigned char*>(img->GetScalarPointer(i, j, 0));
            for (int k = 0; k < 3; ++k) px[k] = static_cast<unsigned char>(255.0 * c[k]);
        }
    }
    m_skyTexture = vtkSmartPointer<vtkTexture>::New();
    m_skyTexture->SetInputData(img);
    m_skyTexture->InterpolateOn();
    m_skyTexture->MipmapOn();

    // image based lighting uses a dimmer copy of the sky: the bright sky alone washes every
    // surface out (ambient light much stronger than the sun)
    vtkNew<vtkImageData> iblImg;
    iblImg->DeepCopy(img);
    {
        auto* px = static_cast<unsigned char*>(iblImg->GetScalarPointer());
        const size_t n = static_cast<size_t>(W) * H * 3;
        for (size_t i = 0; i < n; ++i)
            px[i] = static_cast<unsigned char>(px[i] * 0.55);
    }
    m_iblTexture = vtkSmartPointer<vtkTexture>::New();
    m_iblTexture->SetInputData(iblImg);
    m_iblTexture->InterpolateOn();
    m_iblTexture->MipmapOn();

    m_skybox = vtkSmartPointer<vtkSkybox>::New();
    m_skybox->SetTexture(m_skyTexture);
    m_skybox->SetProjectionToSphere();
    m_skybox->PickableOff();
    m_renderer->SetEnvironmentUp(0.0, 0.0, 1.0);   // sky orientation (default up is +Y)
    m_renderer->SetEnvironmentRight(1.0, 0.0, 0.0);
    m_renderer->AddActor(m_skybox);

    // ---- lights: warm directional sun + weak fill (the sky does the rest through IBL) ----
    m_renderer->AutomaticLightCreationOff();

    m_headLight = vtkSmartPointer<vtkLight>::New();
    m_headLight->SetLightTypeToHeadlight();
    m_headLight->SetIntensity(0.18);
    m_renderer->AddLight(m_headLight);

    m_sunLight = vtkSmartPointer<vtkLight>::New();
    m_sunLight->SetLightTypeToSceneLight();
    m_sunLight->PositionalOff();
    m_sunLight->SetColor(1.0, 0.94, 0.84);
    m_sunLight->SetIntensity(1.35);
    m_renderer->AddLight(m_sunLight);

    m_sunDir[0] = std::cos(sunEl) * std::cos(sunAz);
    m_sunDir[1] = std::cos(sunEl) * std::sin(sunAz);
    m_sunDir[2] = std::sin(sunEl);

    // sky fill light: bluish, from the opposite side and above, so faces turned away from the sun
    // are not black (a cheap stand-in for the light scattered by the sky)
    vtkNew<vtkLight> fill;
    fill->SetLightTypeToSceneLight();
    fill->PositionalOff();
    fill->SetColor(0.72, 0.80, 1.0);
    fill->SetIntensity(0.45);
    fill->SetPosition(-m_sunDir[0], -m_sunDir[1], 1.6);
    fill->SetFocalPoint(0.0, 0.0, 0.0);
    m_renderer->AddLight(fill);

    // ---- screen space ambient occlusion (darkens corners, the base of buildings, under cars) ----
    vtkNew<vtkRenderStepsPass> basicPasses;
    auto ssao = vtkSmartPointer<vtkSSAOPass>::New();
    ssao->SetDelegatePass(basicPasses);
    ssao->SetRadius(1.2);
    ssao->SetBias(0.03);
    ssao->SetKernelSize(32);
    ssao->BlurOn();
    // filmic tone mapping (ACES-like curve) on top: highlights roll off softly, richer contrast
    auto tone = vtkSmartPointer<vtkToneMappingPass>::New();
    tone->SetToneMappingType(vtkToneMappingPass::GenericFilmic);
    tone->SetGenericFilmicDefaultPresets();
    tone->SetUseACES(true);
    tone->SetExposure(1.15);
    tone->SetDelegatePass(ssao);
    m_ssaoPass = tone;

    setupShadows();
}

// Fog and surface detail, added to VTK's own shaders (vtkShaderProperty replacements):
//  - aerial perspective: color -> sky haze with distance (1 - exp(-d * density)), which gives depth
//    and hides the end of the world
//  - multi-octave value noise on the object position modulates the color a little, so large flat
//    surfaces (asphalt, grass, walls) are not perfectly uniform any more
void MainWindow::setAtmosphere(vtkActor* actor, double detail, bool on)
{
    vtkShaderProperty* sp = actor->GetShaderProperty();
    sp->ClearAllShaderReplacements();
    if (!on)
        return;
    sp->AddVertexShaderReplacement("//VTK::CustomUniforms::Dec", true,
                                   "//VTK::CustomUniforms::Dec\nout vec3 atmPos;\n", false);
    sp->AddVertexShaderReplacement("//VTK::CustomEnd::Impl", true,
                                   "//VTK::CustomEnd::Impl\n  atmPos = vertexMC.xyz;\n", false);
    sp->AddFragmentShaderReplacement("//VTK::CustomUniforms::Dec", true,
        "//VTK::CustomUniforms::Dec\n"
        "in vec3 atmPos;\n"
        "float atmHash(vec2 p) { vec3 p3 = fract(vec3(p.xyx) * 0.1031); p3 += dot(p3, p3.yzx + 33.33);\n"
        "  return fract((p3.x + p3.y) * p3.z); }\n"
        "float atmNoise(vec2 p) { vec2 i = floor(p); vec2 f = fract(p); f = f * f * (3.0 - 2.0 * f);\n"
        "  return mix(mix(atmHash(i), atmHash(i + vec2(1.0, 0.0)), f.x),\n"
        "             mix(atmHash(i + vec2(0.0, 1.0)), atmHash(i + vec2(1.0, 1.0)), f.x), f.y); }\n",
        false);
    const std::string impl =
        "  {\n"
        "    float atmDist = 1.0 / gl_FragCoord.w;\n"
        "    vec2 atmP = atmPos.xy + vec2(0.73, -1.31) * atmPos.z;\n"
        "    float atmFine = clamp(1.0 - atmDist / 70.0, 0.0, 1.0);\n"
        "    float atmN = 0.5 * atmNoise(atmP * 0.07) + 0.3 * atmNoise(atmP * 0.55)\n"
        "               + 0.2 * mix(0.5, atmNoise(atmP * 3.3), atmFine);\n"
        "    gl_FragData[0].rgb *= 1.0 + " + std::to_string(detail) + " * (atmN - 0.5) * 2.0;\n"
        // a little contrast (S-curve) and saturation: flat CG colors look less washed out
        "    vec3 atmC = clamp(gl_FragData[0].rgb, 0.0, 1.0);\n"
        "    atmC = mix(atmC, atmC * atmC * (3.0 - 2.0 * atmC), 0.25);\n"
        "    float atmL = dot(atmC, vec3(0.299, 0.587, 0.114));\n"
        "    gl_FragData[0].rgb = mix(vec3(atmL), atmC, 1.12);\n"
        "    float atmF = 1.0 - exp(-max(atmDist - 60.0, 0.0) / 1100.0);\n"
        "    gl_FragData[0].rgb = mix(gl_FragData[0].rgb, vec3(0.80, 0.84, 0.88), clamp(atmF, 0.0, 0.9));\n"
        "  }\n"
        "//VTK::DepthPeeling::Impl\n";
    sp->AddFragmentShaderReplacement("//VTK::DepthPeeling::Impl", true, impl, false);
}

// Planar projected shadows (Blinn, "Me and My (Fake) Shadow", 1988).
// For a directional light l (unit vector toward the light) and the ground plane z = g,
// a point P is projected along -l onto the plane:  P' = P - ((Pz - g) / lz) * l
//     | 1  0  -lx/lz  g*lx/lz |
//     | 0  1  -ly/lz  g*ly/lz |
//     | 0  0   0      g       |
//     | 0  0   0      1       |
// Every occluder is drawn a second time with this matrix as a flat, translucent dark mesh.
// Moving objects use (shadow matrix) * (pose), so shadows follow and even deform with the truck.
void MainWindow::setupShadows()
{
    const double g = m_sim.world.road().groundZ + 0.012;
    const double* l = m_sunDir;

    m_shadowMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
    m_shadowMatrix->Identity();
    m_shadowMatrix->SetElement(0, 2, -l[0] / l[2]);
    m_shadowMatrix->SetElement(0, 3, g * l[0] / l[2]);
    m_shadowMatrix->SetElement(1, 2, -l[1] / l[2]);
    m_shadowMatrix->SetElement(1, 3, g * l[1] / l[2]);
    m_shadowMatrix->SetElement(2, 2, 0.0);
    m_shadowMatrix->SetElement(2, 3, g);

    m_vehicleShadowTransform = vtkSmartPointer<vtkTransform>::New();

    auto addShadow = [this](vtkUnstructuredGrid* grid, vtkLinearTransform* t) {
        vtkNew<vtkDataSetMapper> mapper;
        mapper->SetInputData(grid);   // live grid: deformation shows in the shadow
        mapper->ScalarVisibilityOff();
        auto actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->SetUserTransform(t);
        actor->GetProperty()->SetColor(0.02, 0.03, 0.02);
        actor->GetProperty()->SetOpacity(0.3);
        actor->GetProperty()->LightingOff();
        actor->PickableOff();
        m_renderer->AddActor(actor);
        m_shadowActors.push_back(actor);
    };

    for (int pid : { World::PidCab, World::PidCargo, World::PidTires, World::PidCamera })
        addShadow(m_sim.world.grid(pid), m_vehicleShadowTransform);
    // Static objects: their projected triangles are rasterized ONCE into a shadow mask texture
    // laid on the ground. A translucent projected mesh darkens twice wherever triangles overlap
    // (front + back faces, neighbouring buildings) -> the road under tall buildings turned black.
    // In the mask every covered pixel is simply "in shadow", one layer.
    {
        double b[4];
        m_sim.world.road().bounds(b);
        const double cell = 0.5, margin = 90.0;
        const double x0 = b[0] - margin, y0 = b[2] - margin;
        const int nx = static_cast<int>((b[1] - b[0] + 2 * margin) / cell);
        const int ny = static_cast<int>((b[3] - b[2] + 2 * margin) / cell);
        std::vector<unsigned char> mask(static_cast<size_t>(nx) * ny, 0);
        const double gz = m_sim.world.road().groundZ;

        auto raster = [&](const double a[2], const double bb[2], const double c[2]) {
            const double minx = std::min({ a[0], bb[0], c[0] }), maxx = std::max({ a[0], bb[0], c[0] });
            const double miny = std::min({ a[1], bb[1], c[1] }), maxy = std::max({ a[1], bb[1], c[1] });
            const int i0 = std::max(0, static_cast<int>((minx - x0) / cell));
            const int i1 = std::min(nx - 1, static_cast<int>((maxx - x0) / cell));
            const int j0 = std::max(0, static_cast<int>((miny - y0) / cell));
            const int j1 = std::min(ny - 1, static_cast<int>((maxy - y0) / cell));
            const double area = (bb[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (bb[1] - a[1]);
            if (std::abs(area) < 1e-9)
                return;
            for (int j = j0; j <= j1; ++j)
                for (int i = i0; i <= i1; ++i)
                {
                    const double px = x0 + (i + 0.5) * cell, py = y0 + (j + 0.5) * cell;
                    const double w0 = ((bb[0] - px) * (c[1] - py) - (c[0] - px) * (bb[1] - py)) / area;
                    const double w1 = ((c[0] - px) * (a[1] - py) - (a[0] - px) * (c[1] - py)) / area;
                    if (w0 >= 0 && w1 >= 0 && w0 + w1 <= 1.0)
                        mask[static_cast<size_t>(j) * nx + i] = 255;
                }
        };
        // the road inside the tunnel lies in the shade of the hill
        {
            const RoadModel& rd = m_sim.world.road();
            const double wl = rd.halfWidth + 1.3;
            for (double s = m_sim.world.tunnelFrom(); s < m_sim.world.tunnelTo(); s += 2.0)
            {
                double p0[2], p1[2], p2[2], p3[2];
                rd.point(s, -wl, p0[0], p0[1]);
                rd.point(s, wl, p1[0], p1[1]);
                rd.point(s + 2.0, wl, p2[0], p2[1]);
                rd.point(s + 2.0, -wl, p3[0], p3[1]);
                raster(p0, p1, p2);
                raster(p0, p2, p3);
            }
        }
        for (int pid : { World::PidPoles, World::PidObstacles, World::PidWall, World::PidBuildings,
                         World::PidGlass, World::PidLights, World::PidTrees, World::PidTunnel, World::PidBridge })
        {
            vtkNew<vtkDataSetSurfaceFilter> surf;
            surf->SetInputData(m_sim.world.grid(pid));
            vtkNew<vtkTriangleFilter> tri;
            tri->SetInputConnection(surf->GetOutputPort());
            tri->Update();
            vtkPolyData* pd = tri->GetOutput();
            vtkNew<vtkIdList> ids;
            for (vtkIdType c = 0; c < pd->GetNumberOfCells(); ++c)
            {
                pd->GetCellPoints(c, ids);
                if (ids->GetNumberOfIds() != 3)
                    continue;
                double q[3][2];
                bool above = false;
                for (int k = 0; k < 3; ++k)
                {
                    double P[3];
                    pd->GetPoint(ids->GetId(k), P);
                    const double h = std::max(0.0, P[2] - gz);
                    above = above || h > 0.05;
                    q[k][0] = P[0] - h * l[0] / l[2];
                    q[k][1] = P[1] - h * l[1] / l[2];
                }
                if (above)
                    raster(q[0], q[1], q[2]);
            }
        }

        // the shadow is a flat sheet just above street level: where the ground is cut lower (underpass,
        // lowered streets) it would float in the air -> no baked shadow there
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i)
            {
                unsigned char& m = mask[static_cast<size_t>(j) * nx + i];
                if (!m)
                    continue;
                const double x = x0 + (i + 0.5) * cell, y = y0 + (j + 0.5) * cell;
                if (m_sim.world.groundHole(x, y) || m_sim.world.terrainHeight(x, y) < -0.05)
                    m = 0;
            }

        // soft edges: 3x3 blur, then RGBA texture (black, alpha = shadow strength)
        vtkNew<vtkImageData> img;
        img->SetDimensions(nx, ny, 1);
        img->AllocateScalars(VTK_UNSIGNED_CHAR, 4);
        auto* px = static_cast<unsigned char*>(img->GetScalarPointer());
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i)
            {
                int sum = 0, n = 0;
                for (int dj = -1; dj <= 1; ++dj)
                    for (int di = -1; di <= 1; ++di)
                    {
                        const int ii = i + di, jj = j + dj;
                        if (ii < 0 || jj < 0 || ii >= nx || jj >= ny)
                            continue;
                        sum += mask[static_cast<size_t>(jj) * nx + ii];
                        ++n;
                    }
                unsigned char* o = px + 4 * (static_cast<size_t>(j) * nx + i);
                o[0] = o[1] = o[2] = 8;
                o[3] = static_cast<unsigned char>(0.42 * sum / n);
            }
        vtkNew<vtkTexture> tex;
        tex->SetInputData(img);
        tex->InterpolateOn();
        tex->MipmapOn();
        tex->SetColorModeToDirectScalars();

        vtkNew<vtkPlaneSource> plane;
        // just above the sidewalks so they are shadowed too (the road is 12 cm lower: no visible gap)
        const double z = gz + 0.125;
        plane->SetOrigin(x0, y0, z);
        plane->SetPoint1(x0 + nx * cell, y0, z);
        plane->SetPoint2(x0, y0 + ny * cell, z);
        vtkNew<vtkPolyDataMapper> pm;
        pm->SetInputConnection(plane->GetOutputPort());
        auto actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(pm);
        actor->SetTexture(tex);
        actor->GetProperty()->LightingOff();
        actor->ForceTranslucentOn();
        actor->PickableOff();
        m_renderer->AddActor(actor);
        m_shadowActors.push_back(actor);
    }
    for (size_t i = 0; i < m_sim.npcs.size(); ++i)
    {
        auto t = vtkSmartPointer<vtkTransform>::New();
        m_npcShadowTransforms.push_back(t);
        addShadow(m_sim.world.npcGrid(i), t);
    }
}

void MainWindow::updateShadows()
{
    // project onto the road the car is on (viaduct deck, underpass floor), not onto street level:
    // shift down by the road height, flatten with the sun, shift back up
    auto compose = [this](vtkTransform* out, vtkTransform* pose, double roadZ) {
        out->Identity();
        out->PostMultiply();
        out->Concatenate(pose->GetMatrix());
        out->Translate(0.0, 0.0, -roadZ);
        out->Concatenate(m_shadowMatrix);
        out->Translate(0.0, 0.0, roadZ);
    };
    compose(m_vehicleShadowTransform, m_vehicleTransform, m_sim.vehicle.z());
    for (size_t i = 0; i < m_npcShadowTransforms.size(); ++i)
        compose(m_npcShadowTransforms[i], m_npcTransforms[i], m_sim.npcs[i].z);
}

void MainWindow::setupLidarVisuals()
{
    // ground returns
    vtkNew<vtkPolyDataMapper> cloudMapper;
    cloudMapper->SetInputData(m_sim.lidar.cloud());
    cloudMapper->SetScalarModeToUsePointData();
    cloudMapper->SetColorModeToDirectScalars();
    cloudMapper->ScalarVisibilityOn();

    m_cloudActor = vtkSmartPointer<vtkActor>::New();
    m_cloudActor->SetMapper(cloudMapper);
    m_cloudActor->SetUserTransform(m_vehicleTransform);   // ground returns are in the vehicle frame
    m_cloudActor->GetProperty()->SetPointSize(2.2);
    m_cloudActor->GetProperty()->RenderPointsAsSpheresOn();
    m_cloudActor->GetProperty()->LightingOff();
    m_cloudActor->PickableOff();
    m_overlayRenderer->AddActor(m_cloudActor);

    // obstacle returns: bigger, range-colored
    vtkNew<vtkPolyDataMapper> obstacleMapper;
    obstacleMapper->SetInputData(m_sim.lidar.obstacleCloud());
    obstacleMapper->SetScalarModeToUsePointData();
    obstacleMapper->SetColorModeToDirectScalars();
    obstacleMapper->ScalarVisibilityOn();

    m_obstacleCloudActor = vtkSmartPointer<vtkActor>::New();
    m_obstacleCloudActor->SetMapper(obstacleMapper);
    m_obstacleCloudActor->GetProperty()->SetPointSize(3.5);
    m_obstacleCloudActor->GetProperty()->RenderPointsAsSpheresOn();
    m_obstacleCloudActor->GetProperty()->LightingOff();
    m_obstacleCloudActor->PickableOff();
    m_overlayRenderer->AddActor(m_obstacleCloudActor);

    // (laser beam lines were removed: not needed, and the translucent lines cost ~40 ms per frame)

    // autopilot lookahead target
    vtkNew<vtkSphereSource> sphere;
    sphere->SetRadius(0.25);
    sphere->SetThetaResolution(16);
    sphere->SetPhiResolution(16);
    vtkNew<vtkPolyDataMapper> sphereMapper;
    sphereMapper->SetInputConnection(sphere->GetOutputPort());

    m_lookaheadTransform = vtkSmartPointer<vtkTransform>::New();
    m_lookaheadActor = vtkSmartPointer<vtkActor>::New();
    m_lookaheadActor->SetMapper(sphereMapper);
    m_lookaheadActor->SetUserTransform(m_lookaheadTransform);
    m_lookaheadActor->GetProperty()->SetColor(0.2, 1.0, 0.45);
    m_lookaheadActor->GetProperty()->LightingOff();
    m_lookaheadActor->PickableOff();
    m_lookaheadActor->SetVisibility(0);
    m_overlayRenderer->AddActor(m_lookaheadActor);
}

void MainWindow::setupGhost()
{
    m_ghostTransform = vtkSmartPointer<vtkTransform>::New();

    for (int pid : { World::PidCab, World::PidCargo, World::PidTires })
    {
        auto copy = vtkSmartPointer<vtkUnstructuredGrid>::New();
        copy->DeepCopy(m_sim.world.grid(pid));   // undeformed copy
        m_ghostGrids.push_back(copy);

        vtkNew<vtkDataSetMapper> mapper;
        mapper->SetInputData(copy);
        mapper->ScalarVisibilityOff();

        auto actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->SetUserTransform(m_ghostTransform);
        actor->GetProperty()->SetColor(0.35, 0.9, 1.0);
        actor->GetProperty()->SetOpacity(0.28);
        actor->PickableOff();
        actor->SetVisibility(0);
        m_renderer->AddActor(actor);
        m_ghostActors.push_back(actor);
    }
}

// Cockpit for the first-person view, in the truck's local frame (driver on the left: y ~ 0.27).
// The cab body itself stays visible from inside thanks to backface culling (see setFirstPersonLook):
// its walls face outward, so from the driver's seat only the hood and the outside world remain.
void MainWindow::setupCockpit()
{
    using SceneBuilder::hexBlock;

    // windshield pillar (A-pillar): a thin slanted bar from the hood edge to the roof edge
    auto pillar = [](double y0, double y1) {
        return SceneBuilder::morphedHex(4, 1, 1,
            [=](double u, double v, double w, double p[3]) {
                p[0] = 0.31 + 0.37 * u + 0.05 * w;   // along the windshield slope
                p[1] = y0 + (y1 - y0) * v;
                p[2] = 0.90 + 0.48 * u - 0.03 * w;
            },
            [](int, int, int, double, double, double) { return int(Palette::Trim); });
    };

    auto interior = SceneBuilder::merge({
        hexBlock({ 0.34, 0.05, 0.84 }, { 0.60, 0.95, 0.97 }, 4, 6, 1, Palette::Trim),     // dashboard
        hexBlock({ 0.57, 0.14, 0.97 }, { 0.61, 0.40, 1.04 }, 1, 2, 1, Palette::Window),   // gauges
        hexBlock({ 0.55, 0.46, 0.97 }, { 0.59, 0.62, 1.06 }, 1, 1, 1, Palette::Window),   // screen
        hexBlock({ 0.64, 0.05, 1.32 }, { 0.75, 0.95, 1.38 }, 1, 4, 1, Palette::Trim),     // roof header
        hexBlock({ 0.62, 0.47, 1.27 }, { 0.635, 0.58, 1.31 }, 1, 1, 1, Palette::Trim),    // rear-view mirror
        pillar(0.05, 0.095), pillar(0.905, 0.95) });

    vtkNew<vtkDataSetMapper> mapper;
    mapper->SetInputData(interior);
    mapper->SetScalarModeToUseCellData();
    mapper->SetLookupTable(m_paletteLut);
    mapper->SetScalarRange(-0.5, Palette::Count - 0.5);
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    m_cockpitTransform = vtkSmartPointer<vtkTransform>::New();
    actor->SetUserTransform(m_cockpitTransform);
    actor->PickableOff();
    m_renderer->AddActor(actor);
    m_cockpitActors.push_back(actor);

    // steering wheel: torus rim + one spoke so its rotation is visible
    vtkNew<vtkParametricTorus> torus;
    torus->SetRingRadius(0.16);
    torus->SetCrossSectionRadius(0.018);
    vtkNew<vtkParametricFunctionSource> rim;
    rim->SetParametricFunction(torus);
    rim->SetUResolution(48);
    rim->SetVResolution(10);
    vtkNew<vtkCubeSource> spoke;
    spoke->SetXLength(0.30);
    spoke->SetYLength(0.035);
    spoke->SetZLength(0.02);
    vtkNew<vtkCubeSource> hub;
    hub->SetXLength(0.07);
    hub->SetYLength(0.07);
    hub->SetZLength(0.04);
    vtkNew<vtkAppendPolyData> wheel;
    wheel->AddInputConnection(rim->GetOutputPort());
    wheel->AddInputConnection(spoke->GetOutputPort());
    wheel->AddInputConnection(hub->GetOutputPort());

    vtkNew<vtkPolyDataMapper> wheelMapper;
    wheelMapper->SetInputConnection(wheel->GetOutputPort());
    m_wheelTransform = vtkSmartPointer<vtkTransform>::New();
    auto wheelActor = vtkSmartPointer<vtkActor>::New();
    wheelActor->SetMapper(wheelMapper);
    wheelActor->SetUserTransform(m_wheelTransform);
    wheelActor->GetProperty()->SetColor(0.07, 0.07, 0.08);
    wheelActor->GetProperty()->SetSpecular(0.3);
    wheelActor->PickableOff();
    m_renderer->AddActor(wheelActor);
    m_cockpitActors.push_back(wheelActor);

    setFirstPersonLook(false);
}

void MainWindow::updateCockpit()
{
    // steering wheel: rotate about its own axis by (road-wheel angle x steering ratio 14),
    // tilt its plane toward the driver, place it in front of the seat, then follow the truck
    const double wheelDeg = m_sim.vehicle.steer() * 180.0 / kPi * 14.0;
    m_wheelTransform->Identity();
    m_wheelTransform->PostMultiply();
    m_wheelTransform->RotateZ(wheelDeg);
    m_wheelTransform->RotateY(58.0);
    m_wheelTransform->Translate(m_eyeLocal[0] - 0.32, m_eyeLocal[1], m_eyeLocal[2] - 0.24);
    m_wheelTransform->Concatenate(m_vehicleTransform->GetMatrix());
    // dashboard / pillars were modeled for the truck (eye at 1.02, 1.20): move them with the eye
    m_cockpitTransform->Identity();
    m_cockpitTransform->PostMultiply();
    m_cockpitTransform->Translate(m_eyeLocal[0] - 1.02, 0.0, m_eyeLocal[2] - 1.20);
    m_cockpitTransform->Concatenate(m_vehicleTransform->GetMatrix());
}

void MainWindow::setFirstPersonLook(bool on)
{
    for (auto& a : m_cockpitActors)
        a->SetVisibility(on);
    // from inside, the cab walls are back faces -> culled -> we look through them
    for (int pid : { World::PidCab, World::PidCargo })
        if (vtkActor* body = m_pidActorManager->getActorByPid(pid))
            body->GetProperty()->SetBackfaceCulling(on);
}

void MainWindow::setupHud()
{
    m_hudRenderer = vtkSmartPointer<vtkRenderer>::New();
    m_hudRenderer->SetLayer(2);
    m_hudRenderer->PreserveColorBufferOn();
    m_hudRenderer->InteractiveOff();

    m_hudText = makeText(0.012, 0.982, 14, true);
    m_hudText->GetTextProperty()->SetBackgroundColor(0.03, 0.05, 0.09);
    m_hudText->GetTextProperty()->SetBackgroundOpacity(0.72);
    m_hudText->GetTextProperty()->SetBold(true);
    m_hudText->GetTextProperty()->SetFrame(true);
    m_hudText->GetTextProperty()->SetFrameColor(0.25, 0.55, 1.0);
    m_hudText->GetTextProperty()->SetFrameWidth(1);
    m_hudRenderer->AddActor2D(m_hudText);

    m_bannerText = makeText(0.38, 0.66, 22, true);
    m_bannerText->GetTextProperty()->SetJustificationToCentered();
    m_bannerText->GetTextProperty()->BoldOn();
    m_bannerText->GetTextProperty()->SetBackgroundColor(0.0, 0.0, 0.0);
    m_bannerText->GetTextProperty()->SetBackgroundOpacity(0.55);
    m_bannerText->SetVisibility(0);
    m_hudRenderer->AddActor2D(m_bannerText);

    m_helpText = makeText(0.48, 0.992, 11, true);
    m_helpText->GetTextProperty()->SetJustificationToCentered();
    m_helpText->GetTextProperty()->SetColor(0.85, 0.9, 1.0);
    m_helpText->GetTextProperty()->SetOpacity(0.8);
    m_helpText->GetTextProperty()->SetBackgroundColor(0.03, 0.05, 0.09);
    m_helpText->GetTextProperty()->SetBackgroundOpacity(0.45);
    m_helpText->SetInput("W/S drive   A/D steer   Q/E gear   1-4 vehicle   T autopilot   V/C camera   K damage   "
                         "R reset   F1 help   F3 info");
    m_hudRenderer->AddActor2D(m_helpText);
    m_bannerText->GetTextProperty()->SetFontFamilyToArial();
    m_bannerText->GetTextProperty()->SetFrame(true);
    m_bannerText->GetTextProperty()->SetFrameWidth(2);

    // crash-analysis post-processor color map (LS-PrePost style fringe plot): discrete bands
    // blue (no strain) -> cyan -> green -> yellow -> red -> magenta (material torn / folded)
    m_damageLut = vtkSmartPointer<vtkLookupTable>::New();
    {
        static const double band[12][3] = {
            { 0.00, 0.00, 0.70 }, { 0.00, 0.22, 1.00 }, { 0.00, 0.55, 1.00 }, { 0.00, 0.85, 0.90 },
            { 0.00, 0.80, 0.45 }, { 0.30, 0.90, 0.00 }, { 0.70, 0.95, 0.00 }, { 1.00, 0.90, 0.00 },
            { 1.00, 0.65, 0.00 }, { 1.00, 0.38, 0.00 }, { 0.95, 0.08, 0.00 }, { 0.85, 0.00, 0.45 } };
        m_damageLut->SetNumberOfTableValues(12);
        for (int i = 0; i < 12; ++i)
            m_damageLut->SetTableValue(i, band[i][0], band[i][1], band[i][2], 1.0);
        m_damageLut->SetTableRange(0.0, 30.0);
    }

    m_scalarBar = vtkSmartPointer<vtkScalarBarActor>::New();
    m_scalarBar->SetLookupTable(m_damageLut);
    m_scalarBar->SetTitle("Eff. strain [%]");
    m_scalarBar->SetNumberOfLabels(7);
    m_scalarBar->SetMaximumNumberOfColors(12);
    m_scalarBar->SetLabelFormat("%.0f");
    m_scalarBar->GetPositionCoordinate()->SetCoordinateSystemToNormalizedViewport();
    m_scalarBar->SetPosition(0.015, 0.22);
    m_scalarBar->SetWidth(0.07);
    m_scalarBar->SetHeight(0.42);
    m_scalarBar->SetVisibility(0);
    m_hudRenderer->AddActor2D(m_scalarBar);

    m_renderWindow->AddRenderer(m_hudRenderer);
}

void MainWindow::setupFrontCamera()
{
    m_frontCamRenderer = vtkSmartPointer<vtkRenderer>::New();
    m_frontCamRenderer->SetViewport(0.76, 0.70, 1.0, 1.0);
    m_frontCamRenderer->SetLayer(2);
    m_frontCamRenderer->PreserveColorBufferOff();
    m_frontCamRenderer->InteractiveOff();
    m_frontCamRenderer->GradientBackgroundOn();
    m_frontCamRenderer->SetBackground(0.84, 0.82, 0.78);   // horizon
    m_frontCamRenderer->SetBackground2(0.25, 0.45, 0.78);  // zenith

    // camera-only actors sharing the PID mappers (no mesh edges)
    m_frontCamActors.clear();
    for (int pid : m_pidActorManager->getPidList())
    {
        vtkActor* src = m_pidActorManager->getActorByPid(pid);
        if (!src)
            continue;

        auto camActor = vtkSmartPointer<vtkActor>::New();
        camActor->SetMapper(src->GetMapper());
        camActor->GetProperty()->DeepCopy(src->GetProperty());
        camActor->GetProperty()->SetRepresentationToSurface();
        camActor->GetProperty()->EdgeVisibilityOff();
        if (World::isVehiclePid(pid))
            camActor->SetUserTransform(m_vehicleTransform);

        m_frontCamRenderer->AddActor(camActor);
        m_frontCamActors[pid] = camActor;
    }
    for (auto& npc : m_npcActors)
    {
        vtkNew<vtkActor> a;
        a->SetMapper(npc->GetMapper());
        a->SetUserTransform(npc->GetUserTransform());
        m_frontCamRenderer->AddActor(a);
    }
    vtkNew<vtkActor> terrain;
    terrain->SetMapper(m_terrainActor->GetMapper());
    terrain->GetProperty()->DeepCopy(m_terrainActor->GetProperty());
    m_frontCamRenderer->AddActor(terrain);
    vtkNew<vtkActor> water;
    water->SetMapper(m_waterActor->GetMapper());
    water->GetProperty()->DeepCopy(m_waterActor->GetProperty());
    m_frontCamRenderer->AddActor(water);

    vtkCamera* cam = m_frontCamRenderer->GetActiveCamera();
    cam->SetViewAngle(m_frontCamViewAngle);

    m_frontCamHeadlight = vtkSmartPointer<vtkLight>::New();
    m_frontCamHeadlight->SetLightTypeToHeadlight();
    m_frontCamHeadlight->SetIntensity(1.0);
    m_frontCamRenderer->AutomaticLightCreationOff();
    m_frontCamRenderer->AddLight(m_frontCamHeadlight);

    m_frontCamLabel = makeText(0.03, 0.96, 12, true);
    m_frontCamLabel->SetInput("FRONT CAMERA");
    m_frontCamLabel->GetTextProperty()->SetBackgroundColor(0, 0, 0);
    m_frontCamLabel->GetTextProperty()->SetBackgroundOpacity(0.4);
    m_frontCamRenderer->AddActor2D(m_frontCamLabel);

    m_renderWindow->AddRenderer(m_frontCamRenderer);
}

void MainWindow::setupLidarView()
{
    m_lidarRenderer = vtkSmartPointer<vtkRenderer>::New();
    m_lidarRenderer->SetViewport(0.76, 0.0, 1.0, 0.40);
    m_lidarRenderer->SetLayer(2);
    m_lidarRenderer->PreserveColorBufferOff();
    m_lidarRenderer->InteractiveOff();
    m_lidarRenderer->SetBackground(0.02, 0.03, 0.05);

    m_lidarViewCloudActor = vtkSmartPointer<vtkActor>::New();
    m_lidarViewCloudActor->SetMapper(m_cloudActor->GetMapper());
    m_lidarViewCloudActor->SetUserTransform(m_vehicleTransform);
    m_lidarViewCloudActor->GetProperty()->SetPointSize(2.0);
    m_lidarViewCloudActor->GetProperty()->LightingOff();
    m_lidarRenderer->AddActor(m_lidarViewCloudActor);

    m_lidarViewObstacleActor = vtkSmartPointer<vtkActor>::New();
    m_lidarViewObstacleActor->SetMapper(m_obstacleCloudActor->GetMapper());
    m_lidarViewObstacleActor->GetProperty()->SetPointSize(3.0);
    m_lidarViewObstacleActor->GetProperty()->LightingOff();
    m_lidarRenderer->AddActor(m_lidarViewObstacleActor);

    // ego vehicle footprint
    for (int pid : { World::PidCab, World::PidCargo })
    {
        vtkNew<vtkActor> a;
        a->SetMapper(m_pidActorManager->getActorByPid(pid)->GetMapper());
        a->SetUserTransform(m_vehicleTransform);
        a->GetProperty()->LightingOff();
        m_lidarRenderer->AddActor(a);
    }

    // range rings (10 / 20 / 30 / 40 m) centered on the sensor
    vtkNew<vtkAppendPolyData> rings;
    for (double r : { 10.0, 20.0, 30.0, 40.0 })
    {
        vtkNew<vtkRegularPolygonSource> ring;
        ring->SetNumberOfSides(96);
        ring->SetRadius(r);
        ring->SetCenter(m_sim.lidar.config.mount[0], m_sim.lidar.config.mount[1], -0.05);
        ring->GeneratePolygonOff();
        ring->GeneratePolylineOn();
        rings->AddInputConnection(ring->GetOutputPort());
        m_ringSources.push_back(ring.GetPointer());
    }
    vtkNew<vtkPolyDataMapper> ringMapper;
    ringMapper->SetInputConnection(rings->GetOutputPort());
    vtkNew<vtkActor> ringActor;
    ringActor->SetMapper(ringMapper);
    ringActor->SetUserTransform(m_vehicleTransform);
    ringActor->GetProperty()->SetColor(0.25, 0.32, 0.4);
    ringActor->GetProperty()->LightingOff();
    m_lidarRenderer->AddActor(ringActor);

    m_lidarViewLookahead = vtkSmartPointer<vtkActor>::New();
    m_lidarViewLookahead->SetMapper(m_lookaheadActor->GetMapper());
    m_lidarViewLookahead->SetUserTransform(m_lookaheadTransform);
    m_lidarViewLookahead->GetProperty()->SetColor(0.2, 1.0, 0.45);
    m_lidarViewLookahead->GetProperty()->LightingOff();
    m_lidarRenderer->AddActor(m_lidarViewLookahead);

    auto label = makeText(0.03, 0.97, 12, true);
    label->SetInput("LIDAR TOP VIEW  (rings: 10 m)");
    m_lidarRenderer->AddActor2D(label);

    // intensity legend: the real 8-bit value (Velodyne calibrated reflectivity 0..255) and its color
    {
        vtkNew<vtkLookupTable> lut;
        lut->SetNumberOfTableValues(256);
        lut->SetRange(0.0, 255.0);
        for (int i = 0; i < 256; ++i)
        {
            unsigned char rgb[3];
            LidarSensor::intensityColor(LidarSensor::displayValue(i / 255.0), rgb);
            lut->SetTableValue(i, rgb[0] / 255.0, rgb[1] / 255.0, rgb[2] / 255.0, 1.0);
        }
        lut->Build();
        auto bar = vtkSmartPointer<vtkScalarBarActor>::New();
        bar->SetLookupTable(lut);
        bar->SetOrientationToHorizontal();
        bar->SetTitle("INTENSITY (0-255)");
        bar->SetNumberOfLabels(5);
        bar->SetLabelFormat("%.0f");
        bar->SetPosition(0.03, 0.76);
        bar->SetWidth(0.94);
        bar->SetHeight(0.17);
        bar->SetBarRatio(0.22);
        bar->UnconstrainedFontSizeOn();
        bar->GetTitleTextProperty()->SetFontSize(11);
        bar->GetTitleTextProperty()->ShadowOff();
        bar->GetTitleTextProperty()->ItalicOff();
        bar->GetTitleTextProperty()->BoldOff();
        bar->GetLabelTextProperty()->SetFontSize(10);
        bar->GetLabelTextProperty()->ShadowOff();
        bar->GetLabelTextProperty()->ItalicOff();
        bar->PickableOff();
        m_lidarRenderer->AddActor2D(bar);
    }

    m_lidarRenderer->GetActiveCamera()->ParallelProjectionOn();
    m_lidarRenderer->GetActiveCamera()->SetParallelScale(24.0);

    m_renderWindow->AddRenderer(m_lidarRenderer);
}

void MainWindow::setupMinimap()
{
    m_minimapRenderer = vtkSmartPointer<vtkRenderer>::New();
    m_minimapRenderer->SetViewport(0.0, 0.0, 0.205, 0.34);
    m_minimapRenderer->SetLayer(2);
    m_minimapRenderer->PreserveColorBufferOff();
    m_minimapRenderer->InteractiveOff();
    m_minimapRenderer->SetBackground(0.16, 0.22, 0.14);

    {
        vtkNew<vtkActor> t;
        t->SetMapper(m_terrainActor->GetMapper());
        t->GetProperty()->LightingOff();
        m_minimapRenderer->AddActor(t);
        vtkNew<vtkActor> w;
        w->SetMapper(m_waterActor->GetMapper());
        w->GetProperty()->SetColor(0.15, 0.35, 0.55);
        w->GetProperty()->LightingOff();
        m_minimapRenderer->AddActor(w);
    }
    for (int pid : { World::PidRoad, World::PidMarkings, World::PidWall, World::PidBuildings, World::PidGlass,
                     World::PidBridge })
    {
        vtkNew<vtkActor> a;
        a->SetMapper(m_pidActorManager->getActorByPid(pid)->GetMapper());
        a->GetProperty()->LightingOff();
        m_minimapRenderer->AddActor(a);
    }

    // markers: traffic = white dots, ego = cyan dot (cars would be a few pixels only)
    vtkNew<vtkSphereSource> dot;
    dot->SetRadius(7.0);
    dot->SetCenter(2.0, 0.5, 120.0);   // car center in the local frame, above everything
    vtkNew<vtkPolyDataMapper> dotMapper;
    dotMapper->SetInputConnection(dot->GetOutputPort());
    for (auto& t : m_npcTransforms)
    {
        vtkNew<vtkActor> a;
        a->SetMapper(dotMapper);
        a->SetUserTransform(t);
        a->GetProperty()->SetColor(0.95, 0.95, 0.95);
        a->GetProperty()->LightingOff();
        m_minimapRenderer->AddActor(a);
    }

    vtkNew<vtkSphereSource> egoDot;
    egoDot->SetRadius(12.0);
    vtkNew<vtkPolyDataMapper> egoMapper;
    egoMapper->SetInputConnection(egoDot->GetOutputPort());
    m_minimapEgoTransform = vtkSmartPointer<vtkTransform>::New();
    vtkNew<vtkActor> ego;
    ego->SetMapper(egoMapper);
    ego->SetUserTransform(m_minimapEgoTransform);
    ego->GetProperty()->SetColor(0.2, 0.95, 1.0);
    ego->GetProperty()->LightingOff();
    m_minimapRenderer->AddActor(ego);

    auto label = makeText(0.02, 0.97, 12, true);
    label->SetInput("MAP  2.2 km loop + street network");
    label->GetTextProperty()->SetBackgroundColor(0, 0, 0);
    label->GetTextProperty()->SetBackgroundOpacity(0.4);
    m_minimapRenderer->AddActor2D(label);

    vtkCamera* cam = m_minimapRenderer->GetActiveCamera();
    cam->ParallelProjectionOn();
    double b[4], b2[4];
    m_sim.world.road().bounds(b);
    for (const auto& nr : m_sim.world.net())
    {
        nr.road.bounds(b2);
        b[0] = std::min(b[0], b2[0]); b[1] = std::max(b[1], b2[1]);
        b[2] = std::min(b[2], b2[2]); b[3] = std::max(b[3], b2[3]);
    }
    const double cx = 0.5 * (b[0] + b[1]), cy = 0.5 * (b[2] + b[3]);
    cam->SetFocalPoint(cx, cy, 0.0);
    cam->SetPosition(cx, cy, 800.0);
    cam->SetViewUp(0.0, 1.0, 0.0);
    cam->SetClippingRange(100.0, 1200.0);

    m_renderWindow->AddRenderer(m_minimapRenderer);
}

// =====================================================================
// per-frame updates
// =====================================================================
void MainWindow::updateVehicleActors()
{
    updateSignals();
    m_sim.vehicle.toTransform(m_vehicleTransform);
    for (size_t i = 0; i < m_npcTransforms.size(); ++i)
        m_sim.npcs[i].toTransform(m_npcTransforms[i]);
    updateShadows();

    const double* la = m_sim.pilot.lookaheadPoint();
    m_lookaheadTransform->Identity();
    m_lookaheadTransform->Translate(la[0], la[1], la[2]);

    const bool showLa = m_autopilotAction->isChecked() && !m_sim.crash.happened;
    m_lookaheadActor->SetVisibility(showLa);
    m_lidarViewLookahead->SetVisibility(showLa);

    updateCockpit();
}

void MainWindow::updateSun()
{
    // directional light: only (position - focal point) matters; keep it on the sky's sun
    const double c[3] = { 2.0, 0.5, 0.0 };
    double w[3];
    m_sim.vehicle.localToWorld(c, w);
    m_sunLight->SetFocalPoint(w);
    m_sunLight->SetPosition(w[0] + 100.0 * m_sunDir[0], w[1] + 100.0 * m_sunDir[1], w[2] + 100.0 * m_sunDir[2]);
}

// Quaternion camera.
//   q      = frame * userRot          (frame: per mode, userRot: mouse arcball)
//   eye    = target + q * (zoom * baseOffset)
//   viewUp = q * (0, 0, 1)
// The frame follows the truck's heading (rotation about z) in Chase/Top/Cinematic,
// and is fixed in Free mode, so dragging rotates relative to the truck or to the world.
void MainWindow::updateMainCamera(double dt)
{
    vtkCamera* cam = m_renderer->GetActiveCamera();
    const VehicleModel& v = m_sim.vehicle;

    const double centerLocal[3] = { 2.0, 0.5, 0.0 };
    double center[3];
    v.localToWorld(centerLocal, center);
    const double hq = v.heading() - kPi;   // 0 = truck facing -x (the default)

    auto autoReturn = [&]() {
        // 2 s after the last mouse input, slerp the user rotation back to identity
        if (dt > 0.0 && !m_dragging && m_autoReturnAction && m_autoReturnAction->isChecked() &&
            m_idleClock.isValid() && m_idleClock.elapsed() > 2000)
        {
            const double k = 1.0 - std::exp(-dt * 2.0);
            m_userRot = Quat::slerp(m_userRot, Quat::identity(), k);
            m_zoom += (1.0 - m_zoom) * k;
        }
    };

    // ---- first person: camera at the driver's eye, rigidly attached to the truck ----
    // view direction = (heading frame * head rotation) * forward; the mouse turns the head
    if (m_cameraMode == CamFirst)
    {
        const Quat frame = Quat::aboutZ(hq);   // no smoothing: the cockpit must not swim
        m_lastFrame = frame;
        autoReturn();
        const Quat q = frame * m_userRot;

        const double eyeLocal[3] = { m_eyeLocal[0], m_eyeLocal[1], m_eyeLocal[2] };
        const double fwd[3] = { -1.0, 0.0, -0.08 };   // looking ahead, slightly down
        const double zUp[3] = { 0.0, 0.0, 1.0 };
        double eye[3], dir[3], up[3];
        v.localToWorld(eyeLocal, eye);
        q.rotate(fwd, dir);
        q.rotate(zUp, up);

        cam->SetViewAngle(std::clamp(70.0 * m_zoom, 25.0, 100.0));   // wheel = zoom (FOV)
        cam->SetPosition(eye);
        cam->SetFocalPoint(eye[0] + 10.0 * dir[0], eye[1] + 10.0 * dir[1], eye[2] + 10.0 * dir[2]);
        cam->SetViewUp(up);
        cam->OrthogonalizeViewUp();
        cam->SetClippingRange(0.03, 900.0);
        std::copy(eye, eye + 3, m_camPos);
        m_cameraInitialized = true;
        return;
    }
    cam->SetViewAngle(45.0);

    double target[3] = { center[0], center[1], center[2] + 1.0 };
    double base[3] = { 12.5, 0.0, 3.0 };   // behind (+x when facing -x) and above
    Quat frame = Quat::aboutZ(hq);

    if (m_sim.crash.happened && m_cameraMode != CamFree)
    {
        // crash cam: slow orbit around the impact point (pauses while the user drags)
        if (!m_dragging)
            m_crashOrbit += dt * 0.35;
        // around the car (it slides on after the wreck), in a frame fixed at the moment of the wreck
        target[0] = center[0]; target[1] = center[1]; target[2] = center[2] + 0.6;
        base[0] = 6.5; base[2] = 2.2;
        frame = Quat::aboutZ(m_crashHeading + 0.65 * kPi + m_crashOrbit);
    }
    else
    {
        switch (m_cameraMode)
        {
        case CamTop:
            base[0] = 14.0; base[2] = 42.0;
            break;
        case CamCinematic:
            base[0] = 9.0; base[2] = 1.7;
            target[2] = 0.7;
            frame = Quat::aboutZ(hq + m_sim.time * 0.25);
            break;
        case CamFree:
            frame = m_freeFrame;   // world fixed: the truck drives, the view direction stays
            break;
        case CamChase:
        default:
            break;
        }
    }
    // follow the heading with a slerp instead of instantly: small steering corrections of the
    // autopilot no longer shake the whole view
    if (m_cameraInitialized && dt > 0.0)
        frame = Quat::slerp(m_lastFrame, frame, 1.0 - std::exp(-dt * 5.0));
    m_lastFrame = frame;
    autoReturn();

    const Quat q = frame * m_userRot;
    const double off0[3] = { base[0] * m_zoom, base[1] * m_zoom, base[2] * m_zoom };
    const double zUp[3] = { 0.0, 0.0, 1.0 };
    double off[3], up[3];
    q.rotate(off0, off);
    q.rotate(zUp, up);

    // follow the target smoothly; the rotation itself is applied immediately
    if (!m_cameraInitialized || dt <= 0.0)
    {
        std::copy(target, target + 3, m_camFocal);
        m_cameraInitialized = true;
    }
    else
    {
        lerp3(m_camFocal, target, 1.0 - std::exp(-dt * 8.0));
    }

    for (int i = 0; i < 3; ++i)
    {
        m_camPos[i] = m_camFocal[i] + off[i];
        m_camUp[i] = up[i];
    }
    m_camPos[2] = std::max(m_camPos[2], center[2] + 0.3);   // never below the road
    // under the cover slab of the ring road's underpass: stay below the ceiling
    if (m_sim.surface == DriveSim::Surface::Net && m_sim.netRoad >= 0 && m_cameraMode != CamTop)
    {
        double rs, rl;
        const RoadModel& rr = m_sim.world.net()[m_sim.netRoad].road;
        if (rr.project(m_camPos[0], m_camPos[1], rs, rl) && std::abs(rl) < m_sim.world.netWallLat(m_sim.netRoad) &&
            m_sim.world.netStructure(m_sim.netRoad, rs) == World::NetCovered)
        {
            m_camPos[2] = std::min(m_camPos[2], rr.groundZ + m_sim.world.netCoverTop(m_sim.netRoad, rs) - 1.6);
            m_camFocal[2] = std::min(m_camFocal[2], m_camPos[2] - 0.3);
        }
    }
    // inside the tunnel: stay below the vault (and inside the bore)
    {
        double cs, cl;
        const RoadModel& rd = m_sim.world.road();
        if (rd.project(m_camPos[0], m_camPos[1], cs, cl) && m_sim.world.inTunnel(cs) &&
            std::abs(cl) < rd.halfWidth + 1.0 && m_cameraMode != CamTop)
        {
            m_camPos[2] = std::min(m_camPos[2], rd.groundZ + World::tunnelCeiling(cl, rd.halfWidth) - 0.5);
            m_camFocal[2] = std::min(m_camFocal[2], m_camPos[2] - 0.3);
        }
    }

    cam->SetPosition(m_camPos);
    cam->SetFocalPoint(m_camFocal);
    cam->SetViewUp(m_camUp);
    cam->OrthogonalizeViewUp();
    cam->SetClippingRange(0.1, 900.0);
}

// Arcball (Shoemake 1992): both mouse positions are lifted onto a virtual sphere in view space,
// the shortest rotation between them is the drag quaternion. Rotating the scene by 'drag' is the
// same as rotating the camera by its inverse around the target.
void MainWindow::arcballDrag(const QPoint& from, const QPoint& to)
{
    const double w = m_vtkWidget->width(), h = m_vtkWidget->height();
    const double radius = 0.5 * std::min(w, h);
    auto toSphere = [&](const QPoint& p, double out[3]) {
        arcballVector((p.x() - 0.5 * w) / radius, (0.5 * h - p.y()) / radius, out);
    };
    double a[3], b[3];
    toSphere(from, a);
    toSphere(to, b);

    // view space (right, up, back) -> world
    vtkCamera* cam = m_renderer->GetActiveCamera();
    double dir[3], vup[3];
    cam->GetDirectionOfProjection(dir);
    cam->GetViewUp(vup);
    double right[3] = { dir[1] * vup[2] - dir[2] * vup[1], dir[2] * vup[0] - dir[0] * vup[2],
                        dir[0] * vup[1] - dir[1] * vup[0] };
    const double rn = std::sqrt(right[0] * right[0] + right[1] * right[1] + right[2] * right[2]);
    for (double& r : right) r /= rn;
    const double up[3] = { right[1] * dir[2] - right[2] * dir[1], right[2] * dir[0] - right[0] * dir[2],
                           right[0] * dir[1] - right[1] * dir[0] };
    auto toWorld = [&](const double v[3], double o[3]) {
        for (int i = 0; i < 3; ++i)
            o[i] = right[i] * v[0] + up[i] * v[1] - dir[i] * v[2];
    };
    double wa[3], wb[3];
    toWorld(a, wa);
    toWorld(b, wb);

    const Quat drag = Quat::fromTo(wa, wb);
    const Quat camDelta = drag.conj() * drag.conj();   // inverse, doubled for a livelier feel

    // express the world-space delta in the camera frame: total' = camDelta * frame * user
    m_userRot = (m_lastFrame.conj() * camDelta * m_lastFrame * m_userRot).normalized();
    m_idleClock.restart();

    if (!m_motionTimer->isActive())
    {
        updateMainCamera(0.0);
        m_renderWindow->Render();
    }
}

void MainWindow::resetView()
{
    m_userRot = Quat::identity();
    m_zoom = 1.0;
    m_freeFrame = Quat::aboutZ(m_sim.vehicle.heading() - kPi);
    m_cameraInitialized = false;
    updateMainCamera(0.0);
    m_renderWindow->Render();
}

void MainWindow::pickAt(const QPoint& p)
{
    const double dpr = m_vtkWidget->devicePixelRatioF();
    const int x = static_cast<int>(p.x() * dpr);
    const int y = m_renderWindow->GetSize()[1] - static_cast<int>(p.y() * dpr) - 1;  // VTK: origin bottom-left
    handlePick(x, y);
}

void MainWindow::updateFrontCamera()
{
    if (!m_frontCamRenderer)
        return;

    for (auto& [pid, camActor] : m_frontCamActors)
        if (vtkActor* src = m_pidActorManager->getActorByPid(pid))
            camActor->SetVisibility(src->GetVisibility());

    const VehicleModel& v = m_sim.vehicle;
    const double pitch = m_frontCamPitch * kPi / 180.0;

    const double mountLocal[3] = { m_frontCamMount[0], m_frontCamMount[1], m_frontCamMount[2] };
    const double dirLocal[3] = { -std::cos(pitch), 0.0, -std::sin(pitch) };
    const double upLocal[3]  = { -std::sin(pitch), 0.0,  std::cos(pitch) };

    double pos[3], dir[3], up[3];
    v.localToWorld(mountLocal, pos);
    v.localDirToWorld(dirLocal, dir);
    v.localDirToWorld(upLocal, up);

    vtkCamera* cam = m_frontCamRenderer->GetActiveCamera();
    cam->SetPosition(pos);
    cam->SetFocalPoint(pos[0] + dir[0], pos[1] + dir[1], pos[2] + dir[2]);
    cam->SetViewUp(up);
    cam->SetClippingRange(0.05, 400.0);
}

void MainWindow::updateLidarView()
{
    const VehicleModel& v = m_sim.vehicle;
    const double ahead[3] = { -8.0, 0.5, 0.0 };
    double c[3];
    v.localToWorld(ahead, c);

    vtkCamera* cam = m_lidarRenderer->GetActiveCamera();
    cam->SetFocalPoint(c[0], c[1], 0.0);
    cam->SetPosition(c[0], c[1], 100.0);
    cam->SetViewUp(std::cos(v.heading()), std::sin(v.heading()), 0.0);
    cam->SetClippingRange(1.0, 200.0);
}

void MainWindow::updateMinimap()
{
    const double c[3] = { 2.0, 0.5, 0.0 };
    double w[3];
    m_sim.vehicle.localToWorld(c, w);
    m_minimapEgoTransform->Identity();
    m_minimapEgoTransform->Translate(w[0], w[1], 150.0);

    // fit the whole loop
    const int* sz = m_renderWindow->GetSize();
    if (sz[0] > 0 && sz[1] > 0)
    {
        const double aspect = (0.205 * sz[0]) / std::max(1.0, 0.34 * sz[1]);
        double b[4], b2[4];
        m_sim.world.road().bounds(b);
        for (const auto& nr : m_sim.world.net())
        {
            nr.road.bounds(b2);
            b[0] = std::min(b[0], b2[0]); b[1] = std::max(b[1], b2[1]);
            b[2] = std::min(b[2], b2[2]); b[3] = std::max(b[3], b2[3]);
        }
        const double halfW = 0.5 * (b[1] - b[0]) + 40.0, halfH = 0.5 * (b[3] - b[2]) + 40.0;
        m_minimapRenderer->GetActiveCamera()->SetParallelScale(std::max(halfH, halfW / aspect));
    }
}

void MainWindow::updateGhost()
{
    const bool show = m_ghostAction->isChecked() && !m_ghostPath.empty() &&
                      (m_motionTimer->isActive() || m_sim.time > 0.0);
    for (auto& a : m_ghostActors)
        a->SetVisibility(show);
    if (!show)
        return;

    const auto& p = m_ghostPath[std::min(m_ghostIndex, m_ghostPath.size() - 1)];
    m_ghostTransform->Identity();
    m_ghostTransform->PostMultiply();
    m_ghostTransform->Translate(-VehicleModel::kPivotX, -VehicleModel::kPivotY, 0.0);
    m_ghostTransform->RotateY(p.pitch * 180.0 / kPi);
    m_ghostTransform->RotateZ((p.heading - kPi) * 180.0 / kPi);
    m_ghostTransform->Translate(p.x, p.y, p.z);
}

void MainWindow::updateHud()
{
    const VehicleModel& v = m_sim.vehicle;
    const bool ap = m_autopilotAction->isChecked();
    static const char* camNames[] = { "3RD PERSON", "TOP", "FREE", "CINEMATIC", "1ST PERSON" };

    auto fmtDist = [](double d) {
        return d > 1e8 ? QString(" clear") : QString("%1 m").arg(d, 5, 'f', 1);
    };

    QString state;
    if (m_sim.crash.happened)
        state = "CRASHED";
    else if (ap)
        state = m_sim.pilot.stateName();
    else
        state = std::abs(v.speed()) > 0.1 ? "DRIVING" : "IDLE";

    QString text;
    text += QString("  %1  \u00b7  %2  \n").arg(ap ? "AUTOPILOT" : "MANUAL").arg(state);
    text += QString("  CAM %1   \u00b7   %2  %3 t  \n").arg(camNames[m_cameraMode])
                .arg(VehicleShapes::spec(m_sim.vehicleType()).name).arg(m_sim.vehicle.mass / 1000.0, 0, 'f', 2);
    if (m_sim.impacts > 0)
        text += QString("  DAMAGE %1%   impacts %2  \n").arg(std::min(999.0, m_sim.damagePct), 0, 'f', 0)
                    .arg(m_sim.impacts);
    if (m_hudDebug)
    {
        text += QString("  STEER %1 deg   %2 Nm  \n").arg(v.steer() * 180.0 / kPi, 0, 'f', 1)
                    .arg(v.engineTorque(), 0, 'f', 0);
        text += QString("  DIST %1 m   TIME %2 s  \n").arg(m_sim.distance, 0, 'f', 0).arg(m_sim.time, 0, 'f', 1);
        text += QString("  LIDAR VLP-32C  %1 pts/s  %2 pts/rev  %3 ms  \n")
                    .arg(QString::number(m_sim.lidar.pointsPerSecond() / 1000.0, 'f', 0) + "k")
                    .arg(m_sim.lidar.totalPoints())
                    .arg(m_sim.lidar.lastScanMs(), 0, 'f', 1);
    }
    {
        const auto& sg = m_sim.signalAhead;
        static const char* lightNames[] = { "GREEN", "YELLOW", "RED", "RED + LEFT ARROW" };
        if (sg.junction >= 0 && sg.distance < 150.0)
            text += QString("  SIGNAL %1  %2 m  (%3 s)%4  \n")
                        .arg(lightNames[static_cast<int>(sg.light)])
                        .arg(std::max(0.0, sg.distance), 0, 'f', 0)
                        .arg(sg.remaining, 0, 'f', 0)
                        .arg(m_sim.redLightViolations ? QString("   violations %1").arg(m_sim.redLightViolations)
                                                      : QString());
    }
    if (ap)
        text += QString("  LANE  L1 %1   L2 %2   L3 %3   -> L%4  \n")
                    .arg(fmtDist(m_sim.pilot.laneFreeDistance(0)))
                    .arg(fmtDist(m_sim.pilot.laneFreeDistance(1)))
                    .arg(fmtDist(m_sim.pilot.laneFreeDistance(2)))
                    .arg(m_sim.pilot.targetLane() + 1);
    if (m_recorder.active())
        text += QString("  * REC  %1 frames  %2 MB  queue %3%4  \n")
                    .arg(m_recorder.frames()).arg(m_recorder.megabytesWritten(), 0, 'f', 0)
                    .arg(m_recorder.pending())
                    .arg(m_recorder.dropped() ? QString("  dropped %1").arg(m_recorder.dropped()) : QString());
    if (m_hudDebug)
    {
        text += QString("  FPS %1   frame %2 / %3 ms   gap %4 ms  \n")
                    .arg(m_fps, 0, 'f', 0).arg(m_workMsAvg, 0, 'f', 1).arg(m_workMsMax, 0, 'f', 1)
                    .arg(m_intervalMsMax, 0, 'f', 0);
        text += QString("  sim %1  cloud %2  render %3 ms  ")
                    .arg(m_simMs, 0, 'f', 1).arg(m_cloudMs, 0, 'f', 1).arg(m_renderMs, 0, 'f', 1);
    }
    else
        text += QString("  %1 FPS  ").arg(m_fps, 0, 'f', 0);
    m_hudText->SetInput(text.toUtf8().constData());

    // banner
    if (m_sim.crash.happened)
    {
        const auto& r = m_sim.report;
        m_bannerText->SetInput(QString(" WRECKED  %1 km/h  |  %5 g  |  %2 (%3/4)  |  crush %4 cm ")
                                   .arg(r.speedKmh, 0, 'f', 0)
                                   .arg(r.gradeEn).arg(r.grade)
                                   .arg(100.0 * m_sim.deformer.maxDisplacement(), 0, 'f', 0)
                                   .arg(r.peakG, 0, 'f', 0)
                                   .toUtf8().constData());
        static const double gradeRgb[5][3] = { { 1, 1, 1 }, { 0.3, 0.9, 0.4 }, { 1.0, 0.85, 0.1 },
                                               { 1.0, 0.55, 0.2 }, { 1.0, 0.25, 0.2 } };
        const double* c = gradeRgb[std::clamp(r.grade, 0, 4)];
        m_bannerText->GetTextProperty()->SetColor(c[0], c[1], c[2]);
        m_bannerText->SetVisibility(1);
    }
    else if (m_sim.impacts > 0 && m_sim.time - m_lastImpactTime < 2.5)
    {
        const auto& im = m_sim.lastImpact;
        m_bannerText->SetInput(QString(" IMPACT %1 km/h  |  %2 g  |  DAMAGE %3% ")
                                   .arg(im.speedKmh, 0, 'f', 0).arg(im.peakG, 0, 'f', 0)
                                   .arg(m_sim.damagePct, 0, 'f', 0).toUtf8().constData());
        const double d = m_sim.damagePct;
        m_bannerText->GetTextProperty()->SetColor(d < 15 ? 0.3 : 1.0, d < 15 ? 0.9 : d < 40 ? 0.85 : 0.45,
                                                  d < 15 ? 0.4 : 0.15);
        m_bannerText->SetVisibility(1);
    }
    else if (m_sim.time - m_sim.lastViolationTime < 2.5)
    {
        m_bannerText->SetInput(" RED LIGHT VIOLATION ");
        m_bannerText->GetTextProperty()->SetColor(1.0, 0.25, 0.2);
        m_bannerText->SetVisibility(1);
    }
    else if (ap && m_sim.pilot.state() == AutoPilot::State::Aeb)
    {
        m_bannerText->SetInput(" AEB  EMERGENCY BRAKE ");
        m_bannerText->GetTextProperty()->SetColor(1.0, 0.65, 0.1);
        m_bannerText->SetVisibility(1);
    }
    else
    {
        m_bannerText->SetVisibility(0);
    }

    // Korean explanation in the status bar
    double egoS, egoLat;
    m_sim.egoRoadPosition(egoS, egoLat);
    const QString section = ko(u8"📍 ") + (m_sim.surface == DriveSim::Surface::Main
                                               ? ko(m_sim.world.sectionNameKo(egoS))
                                               : ko(m_sim.surfaceNameKo()));
    QString st;
    if (m_sim.crash.happened)
        st = QString(ko(u8"💥 충돌! 파손 등급 %1 — 오른쪽 '충돌 분석' 탭 참고. 차체를 클릭해 보세요. R = 다시 시작"))
                 .arg(ko(m_sim.report.gradeKo));
    else if (ap)
        st = ko(u8"🤖 ") + ko(m_sim.pilot.stateDescriptionKo());
    else if (m_sim.time - m_sim.lastViolationTime < 4.0)
        st = QString(ko(u8"🚨 신호 위반! 빨간불에 정지선을 넘었습니다 (이번 주행 %1회)")).arg(m_sim.redLightViolations);
    else if (!m_motionTimer->isActive())
        st = ko(u8"⏸ 대기 중 — W로 출발, T로 자율주행");
    else
        st = ko(u8"🚗 수동 운전 — W/S 가속·감속, A/D 핸들, T 자율주행 (빨간불에는 정지선 앞에서 멈추세요)");
    if (section != m_lastSectionText)
    {
        m_sectionLabel->setText(section);
        m_lastSectionText = section;
    }
    if (st != m_lastStateText)
    {
        m_stateLabel->setText(st);
        m_lastStateText = st;
    }
}

void MainWindow::refreshAll()
{
    m_sim.lidar.buildOutputs();
    updateVehicleActors();
    updateGhost();
    updateSun();
    updateMainCamera(0.0);
    updateFrontCamera();
    updateLidarView();
    updateMinimap();
    updateHud();
    m_renderWindow->Render();
}

// =====================================================================
// simulation loop
// =====================================================================
VehicleModel::Command MainWindow::manualCommand() const
{
    const bool up    = m_keys.contains(Qt::Key_W) || m_keys.contains(Qt::Key_Up) ||
                       (m_demo.enabled && m_demo.throttle);
    const bool down  = m_keys.contains(Qt::Key_S) || m_keys.contains(Qt::Key_Down);
    const bool left  = m_keys.contains(Qt::Key_A) || m_keys.contains(Qt::Key_Left);
    const bool right = m_keys.contains(Qt::Key_D) || m_keys.contains(Qt::Key_Right);
    const bool space = m_keys.contains(Qt::Key_Space);

    const VehicleModel& v = m_sim.vehicle;
    VehicleModel::Command cmd;
    cmd.throttle = up ? 1.0 : 0.0;
    if (down)
    {
        if (v.speed() > 0.3) cmd.brake = 1.0;
        else cmd.throttle = -1.0;
    }
    cmd.handbrake = space;   // Space = hand brake (rear wheels lock -> the rear slides out)

    const double s = (left ? 1.0 : 0.0) - (right ? 1.0 : 0.0) + (m_demo.enabled ? m_demo.steer : 0.0);
    cmd.steer = s * v.maxSteer / (1.0 + 0.06 * std::abs(v.speed()));
    return cmd;
}

void MainWindow::updateMotion()
{
    // Real elapsed time since the last frame. The timer fires as often as the display allows
    // (Render() waits for vsync), and the simulation advances by exactly the time that passed,
    // in fixed 10 ms sub-steps. A fixed 20 ms step on a 60 Hz display showed some positions for
    // one refresh and others for two, which looked like shaking.
    double dt = m_motionDt;
    QElapsedTimer work;
    work.start();

    if (m_frameClock.isValid())
    {
        const double ms = std::max<double>(0.5, m_frameClock.nsecsElapsed() / 1e6);
        m_frameClock.restart();
        m_fps = 0.9 * m_fps + 0.1 * (1000.0 / ms);
        m_intervalMsMaxWindow = std::max(m_intervalMsMaxWindow, ms);
        dt = std::clamp(ms / 1000.0, 0.001, 0.05);
    }
    else
    {
        m_frameClock.start();
    }

    // LiDAR results of the scan that ran in the background during the last render
    m_sim.lidar.sync();
    {
        const int frame = m_recorder.poll(m_sim);
        if (frame >= 0 && m_recCamAction->isChecked())
            m_pendingImageFrame = frame;
    }
    // point cloud geometry (up to ~58k points) is rebuilt at ~30 Hz, not every frame
    if (m_sim.lidar.outputsDirty() && (!m_cloudClock.isValid() || m_cloudClock.elapsed() >= 33))
    {
        const qint64 tc = work.nsecsElapsed();
        m_sim.lidar.buildOutputs();
        m_cloudClock.restart();
        m_cloudMs = 0.9 * m_cloudMs + 0.1 * ((work.nsecsElapsed() - tc) / 1e6);
    }

    if (!m_sim.crash.happened)
    {
        if (m_autopilotAction->isChecked() && !m_sim.canAutopilot())
        {
            m_autopilotAction->setChecked(false);
            statusBar()->showMessage(ko(u8"자율주행은 큰 순환도로(차선 위)에서만 켤 수 있습니다 — 직접 운전하세요"), 5000);
        }
        m_sim.autopilotEnabled = m_autopilotAction->isChecked();
        m_sim.pilot.cruiseSpeed = m_cruiseSpin->value() / 3.6;

        constexpr double kStep = 0.01;
        m_simAccum += dt;
        int steps = 0;
        while (m_simAccum >= kStep && steps < 6 && !m_sim.crash.happened)
        {
            m_sim.step(kStep, manualCommand());
            m_simAccum -= kStep;
            ++steps;
            if (m_ghostIndex + 1 < m_ghostPath.size())
                ++m_ghostIndex;
        }
        if (steps == 6)
            m_simAccum = 0.0;   // fell far behind: drop the backlog instead of spiraling

        if (m_sim.impactSerial != m_seenImpact)
        {
            m_seenImpact = m_sim.impactSerial;
            m_recorder.logCrash(m_sim);
            onImpact();
        }
        if (m_sim.crash.happened)
            startCrashAnimation();
    }
    else
    {
        m_sim.animate(dt);   // the wreck keeps crumpling / settling
    }
    updateDamageView();
    if (m_sim.impacts > 0 && m_crashDock->isVisible() &&
        (!m_reportClock.isValid() || m_reportClock.elapsed() > 500))
        showCrashReport();
    // fire the LiDAR columns of these steps on the worker threads while this frame renders
    m_sim.lidar.kick();
    m_simMs = 0.9 * m_simMs + 0.1 * (work.nsecsElapsed() / 1e6);



    updateVehicleActors();
    updateGhost();
    updateSun();
    updateMainCamera(dt);
    updateGauges();
    if (m_shakeTime >= 0.0)
        m_shakeTime += dt;
    applyCameraShake();
    updateDebris(dt);
    updateFrontCamera();
    updateLidarView();
    updateMinimap();
    // HUD text is re-rasterized by FreeType on every change: 10 Hz is plenty
    if (!m_hudClock.isValid() || m_hudClock.elapsed() >= 100 || m_sim.crash.happened)
    {
        updateHud();
        m_hudClock.restart();
    }

    const qint64 tRender = work.nsecsElapsed();
    m_renderWindow->Render();
    if (m_pendingImageFrame >= 0)
    {
        captureCameraImage(m_pendingImageFrame);
        m_pendingImageFrame = -1;
    }
    m_renderMs = 0.9 * m_renderMs + 0.1 * ((work.nsecsElapsed() - tRender) / 1e6);

    // stutter diagnostics: average / worst work time and worst tick interval over ~1 s
    const double workMs = work.nsecsElapsed() / 1e6;
    m_workMsAvg = 0.95 * m_workMsAvg + 0.05 * workMs;
    m_workMsMaxWindow = std::max(m_workMsMaxWindow, workMs);
    if (++m_workWindowTicks >= 60)
    {
        m_workMsMax = m_workMsMaxWindow;
        m_intervalMsMax = m_intervalMsMaxWindow;
        m_workMsMaxWindow = m_intervalMsMaxWindow = 0.0;
        m_workWindowTicks = 0;
    }

    if (m_demo.enabled)
    {
        m_demoTime += dt;
        while (m_demoShot < m_demo.shots.size() && m_demoTime >= m_demo.shots[m_demoShot].first)
        {
            // "..._ui.png": the whole window (toolbars, docks, status bar), otherwise the 3D view only
            const QString path = m_demo.shots[m_demoShot++].second;
            if (path.endsWith(QStringLiteral("_ui.png")))
            {
                m_renderWindow->Render();
                grab().save(path);
            }
            else
                saveScreenshot(path);
        }
        if (!m_demo.framesDir.isEmpty() && m_demoTime >= m_demo.framesFrom &&
            m_demoTime <= m_demo.framesFrom + m_demo.framesDur)
        {
            const int idx = static_cast<int>((m_demoTime - m_demo.framesFrom) * 15.0);
            if (idx > m_lastFrameIdx)
            {
                m_lastFrameIdx = idx;
                saveScreenshot(QString("%1/f%2.png").arg(m_demo.framesDir).arg(idx, 4, 10, QChar('0')));
            }
        }
        if (m_demo.quitAfter > 0.0 && m_demoTime >= m_demo.quitAfter)
        {
            m_recordAction->setChecked(false);
            close();
        }
    }
}

void MainWindow::runDemo(const DemoOptions& demo)
{
    m_demo = demo;
    m_demo.enabled = true;
    m_demoTime = 0.0;
    m_demoShot = 0;
    std::sort(m_demo.shots.begin(), m_demo.shots.end());

    m_shadowAction->setChecked(demo.shadows);
    m_pbrAction->setChecked(demo.pbr);
    m_hqAction->setChecked(demo.hq);
    if (demo.edges)
        m_edgesAction->setChecked(true);
    if (demo.selectPid > 0)
    {
        m_pidActorManager->setSelectedPid(demo.selectPid);
        selectTreeItemByPid(demo.selectPid);
    }
    if (demo.hideLidar)
        m_lidarAction->setChecked(false);
    m_cameraCombo->setCurrentIndex(demo.camera);

    if (demo.vehicle != 0)
    {
        QSignalBlocker block(m_vehicleCombo);
        m_vehicleCombo->setCurrentIndex(demo.vehicle);
        m_sim.setVehicleType(demo.vehicle);
        applyVehicleSpec();
    }
    if (demo.startPlace != 0)
    {
        QSignalBlocker block(m_startCombo);
        m_startCombo->setCurrentIndex(demo.startPlace);
        applyStartPlace(demo.startPlace);
        m_sim.reset();
        m_cameraInitialized = false;
    }
    if (demo.orbitYawDeg != 0.0 || demo.orbitPitchDeg != 0.0)
    {
        // same result as dragging: yaw about z, pitch about the camera's sideways axis (y in the frame)
        const double yAxis[3] = { 0.0, 1.0, 0.0 };
        m_userRot = Quat::aboutZ(demo.orbitYawDeg * kPi / 180.0) *
                    Quat::axisAngle(yAxis, -demo.orbitPitchDeg * kPi / 180.0);
    }
    if (demo.zoom != 1.0)
        m_zoom = std::clamp(demo.zoom, 0.25, 12.0);
    if (demo.autopilot)
        m_autopilotAction->setChecked(true);
    if (demo.record)
        m_recordAction->setChecked(true);
    startMotion();
}

void MainWindow::saveScreenshot(const QString& path)
{
    if (m_demo.enabled)
    {
        // timing next to the image (path.txt) for automated performance checks
        QFile f(path + ".txt");
        if (f.open(QIODevice::WriteOnly | QIODevice::Text))
            f.write(QString("fps %1 frame_avg %2 frame_max %3 gap_max %4 sim %5 cloud %6 render %7 pts %8\n")
                        .arg(m_fps, 0, 'f', 1).arg(m_workMsAvg, 0, 'f', 1).arg(m_workMsMax, 0, 'f', 1)
                        .arg(m_intervalMsMax, 0, 'f', 1).arg(m_simMs, 0, 'f', 1).arg(m_cloudMs, 0, 'f', 1)
                        .arg(m_renderMs, 0, 'f', 1).arg(m_sim.lidar.totalPoints()).toUtf8());
    }
    m_renderWindow->Render();
    if (path.endsWith("_win.png"))
    {
        grab().save(path);  // whole window incl. Qt panels
        return;
    }
    vtkNew<vtkWindowToImageFilter> grab;
    grab->SetInput(m_renderWindow);
    grab->ReadFrontBufferOff();
    grab->Update();

    vtkNew<vtkPNGWriter> writer;
    writer->SetFileName(path.toUtf8().constData());  // VTK expects UTF-8 file names on Windows
    writer->SetInputConnection(grab->GetOutputPort());
    writer->Write();
    statusBar()->showMessage(ko(u8"저장됨: ") + path, 4000);
}

// =====================================================================
// Data acquisition
// =====================================================================
void MainWindow::setRecording(bool on)
{
    if (!on)
    {
        if (!m_recorder.active())
            return;
        const int frames = m_recorder.frames();
        const int dropped = m_recorder.dropped();
        statusBar()->showMessage(ko(u8"데이터 저장 마무리 중…"));
        m_recorder.stop();   // waits for the pending writes
        if (m_frontCamLabel)
            m_frontCamLabel->SetVisibility(true);
        const std::string err = m_recorder.lastError();
        statusBar()->showMessage(
            err.empty()
                ? ko(u8"녹화 완료: %1 프레임 (%2 MB)%3 → %4  ('📂 데이터 폴더'로 열기)")
                      .arg(frames).arg(m_recorder.megabytesWritten(), 0, 'f', 0)
                      .arg(dropped ? ko(u8", 디스크가 느려 %1 프레임 건너뜀").arg(dropped) : QString())
                      .arg(QDir::toNativeSeparators(m_lastDataDir))
                : ko(u8"녹화 중 오류: ") + QString::fromStdString(err),
            15000);
        updateHud();
        return;
    }

    const QString base = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation) + "/LidarDriveData";
    const QString dir = base + "/run_" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");

    // the front camera inset as seen in the window = the recorded image
    DataRecorder::CameraInfo cam;
    const int* sz = m_renderWindow->GetSize();
    cam.width = static_cast<int>(0.24 * sz[0]);
    cam.height = static_cast<int>(0.30 * sz[1]);
    cam.vfovDeg = m_frontCamViewAngle;
    for (int k = 0; k < 3; ++k)
        cam.mount[k] = m_frontCamMount[k];
    cam.pitchDownDeg = m_frontCamPitch;

    DataRecorder::Options opt;
    opt.pcd = m_recPcdAction->isChecked();
    if (!m_recorder.start(dir.toUtf8().toStdString(), m_sim, opt, m_recCamAction->isChecked() ? &cam : nullptr))
    {
        statusBar()->showMessage(ko(u8"녹화를 시작할 수 없음: ") + QString::fromStdString(m_recorder.lastError()),
                                 10000);
        QSignalBlocker block(m_recordAction);
        m_recordAction->setChecked(false);
        return;
    }
    m_lastDataDir = dir;
    if (m_frontCamLabel && m_recCamAction->isChecked())
        m_frontCamLabel->SetVisibility(false);   // keep the label out of the recorded images
    statusBar()->showMessage(ko(u8"⏺ 녹화 중 → ") + QDir::toNativeSeparators(dir), 8000);
    if (!m_motionTimer->isActive() && !m_sim.crash.happened)
        startMotion();
    updateHud();
    m_vtkWidget->setFocus();
}

void MainWindow::captureCameraImage(int frame)
{
    // read back the front camera viewport of the frame that was just rendered
    vtkNew<vtkWindowToImageFilter> grab;
    grab->SetInput(m_renderWindow);
    grab->SetViewport(0.76, 0.70, 1.0, 1.0);
    grab->SetInputBufferTypeToRGB();
    grab->ReadFrontBufferOff();
    grab->ShouldRerenderOff();
    grab->Update();
    vtkImageData* img = grab->GetOutput();
    int dims[3];
    img->GetDimensions(dims);
    auto* src = static_cast<unsigned char*>(img->GetScalarPointer());
    if (!src || dims[0] <= 0 || dims[1] <= 0)
        return;
    std::vector<unsigned char> rgb(src, src + static_cast<size_t>(dims[0]) * dims[1] * 3);
    m_recorder.addImage(frame, dims[0], dims[1], std::move(rgb));
}

void MainWindow::applyStartPlace(int index)
{
    index = std::clamp(index, 0, 5);
    m_sim.startNetRoad = -1;
    if (index >= 4)
    {
        // just out of the junction before the viaduct over the highway / before the underpass
        const World& w = m_sim.world;
        const int road = index == 4 ? w.viaductRoad() : w.underpassRoad();
        if (road >= 0)
        {
            m_sim.startNetRoad = road;
            m_sim.startNetS = w.nodes()[w.net()[road].nodeA].radius + 6.0;
        }
        if (m_autopilotAction->isChecked())
            m_autopilotAction->setChecked(false);
    }
    else
        m_sim.startS = m_sim.world.placeS(index);
}

void MainWindow::setStartPlace(int index)
{
    applyStartPlace(index);
    resetMotion();
    m_vtkWidget->setFocus();
}

void MainWindow::applyVehicleSpec()
{
    const VehicleShapes::Spec& sp = VehicleShapes::spec(m_sim.vehicleType());
    for (int k = 0; k < 3; ++k)
    {
        m_eyeLocal[k] = sp.eye[k];
        m_frontCamMount[k] = sp.frontCam[k];
    }
    for (auto& ring : m_ringSources)
        ring->SetCenter(sp.lidarMount[0], sp.lidarMount[1], -0.05);
    // the ghost is the previous run of the previous vehicle: start a fresh one
    const int pids[] = { World::PidCab, World::PidCargo, World::PidTires };
    for (size_t i = 0; i < m_ghostGrids.size() && i < 3; ++i)
    {
        m_ghostGrids[i]->DeepCopy(m_sim.world.grid(pids[i]));
        m_ghostGrids[i]->Modified();
    }
    m_ghostPath.clear();
    m_ghostIndex = 0;
    m_pidActorManager->clearSelection();
}

void MainWindow::setVehicleType(int type)
{
    const bool wasRunning = m_motionTimer->isActive();
    m_motionTimer->stop();
    if (m_recorder.active())
        m_recordAction->setChecked(false);   // the dataset's calibration belongs to one vehicle
    m_sim.setVehicleType(std::clamp(type, 0, VehicleShapes::TypeCount - 1));
    applyVehicleSpec();
    showDamage(false);
    m_damageShown = false;
    m_lastImpactTime = -100.0;
    m_seenImpact = m_sim.impactSerial;
    clearDebris();
    m_crashAnimTime = -1.0;
    m_cameraInitialized = false;
    m_keys.clear();
    refreshAll();
    const VehicleShapes::Spec& sp = VehicleShapes::spec(m_sim.vehicleType());
    statusBar()->showMessage(QString(ko(u8"차종: %1 — %2톤, %3 kW, 최고 %4 km/h"))
                                 .arg(ko(sp.nameKo)).arg(sp.mass / 1000.0, 0, 'f', 2)
                                 .arg(sp.power / 1000.0, 0, 'f', 0).arg(sp.maxSpeed * 3.6, 0, 'f', 0), 6000);
    if (wasRunning)
        startMotion();
    m_vtkWidget->setFocus();
}

void MainWindow::setupSignals()
{
    // ---- traffic light lamps: one polydata, colors per cell (lit / dark), drawn without lighting
    m_lampPoly = vtkSmartPointer<vtkPolyData>::New();
    vtkNew<vtkPoints> pts;
    vtkNew<vtkCellArray> polys;
    vtkNew<vtkUnsignedCharArray> colors;
    colors->SetNumberOfComponents(3);
    colors->SetName("LampColors");
    m_lampCells.clear();
    for (const SignalLamp& L : m_sim.world.signalLamps())
    {
        const int first = static_cast<int>(polys->GetNumberOfCells());
        size_t k = 0;
        for (int n : L.polySizes)
        {
            std::vector<vtkIdType> ids;
            for (int i = 0; i < n; ++i, ++k)
                ids.push_back(pts->InsertNextPoint(L.pts[k][0], L.pts[k][1], L.pts[k][2]));
            polys->InsertNextCell(n, ids.data());
            const unsigned char off[3] = { 30, 26, 24 };
            colors->InsertNextTypedTuple(off);
        }
        m_lampCells.push_back({ first, static_cast<int>(L.polySizes.size()) });
    }
    m_lampPoly->SetPoints(pts);
    m_lampPoly->SetPolys(polys);
    m_lampPoly->GetCellData()->SetScalars(colors);
    m_lampLit.assign(m_lampCells.size(), -1);

    auto lampActor = [&](vtkRenderer* ren) {
        vtkNew<vtkPolyDataMapper> mapper;
        mapper->SetInputData(m_lampPoly);
        mapper->SetScalarModeToUseCellData();
        mapper->SetColorModeToDirectScalars();
        auto a = vtkSmartPointer<vtkActor>::New();
        a->SetMapper(mapper);
        a->GetProperty()->LightingOff();
        a->PickableOff();
        ren->AddActor(a);
    };
    lampActor(m_renderer);
    if (m_frontCamRenderer)
        lampActor(m_frontCamRenderer);

    // ---- tunnel light strips (emissive)
    if (vtkUnstructuredGrid* g = m_sim.world.tunnelLights())
    {
        for (vtkRenderer* ren : { m_renderer.GetPointer(), m_frontCamRenderer.GetPointer() })
        {
            if (!ren)
                continue;
            vtkNew<vtkDataSetMapper> mapper;
            mapper->SetInputData(g);
            mapper->ScalarVisibilityOff();
            vtkNew<vtkActor> a;
            a->SetMapper(mapper);
            a->GetProperty()->SetColor(1.0, 0.88, 0.62);
            a->GetProperty()->LightingOff();
            a->PickableOff();
            ren->AddActor(a);
        }
    }
    updateSignals();
}

void MainWindow::updateSignals()
{
    if (!m_lampPoly)
        return;
    const auto& lamps = m_sim.world.signalLamps();
    auto* colors = vtkUnsignedCharArray::SafeDownCast(m_lampPoly->GetCellData()->GetScalars());
    static const unsigned char lit[4][3] = { { 255, 40, 30 }, { 255, 190, 20 }, { 60, 255, 110 }, { 60, 255, 140 } };
    static const unsigned char dark[4][3] = { { 60, 18, 16 }, { 60, 46, 14 }, { 16, 52, 26 }, { 16, 52, 32 } };
    // blinking is not used in Korea for these signals; aspects are steady
    bool changed = false;
    for (size_t i = 0; i < lamps.size(); ++i)
    {
        const SignalLamp& L = lamps[i];
        const Light a = m_sim.world.signalAt(L.junction, L.dir, m_sim.time);
        bool on = false;
        switch (L.slot)
        {
        case 0: on = a == Light::Red || a == Light::RedArrow; break;
        case 1: on = a == Light::Yellow; break;
        case 2: on = a == Light::RedArrow; break;
        case 3: on = a == Light::Green; break;
        }
        if (m_lampLit[i] == int(on))
            continue;
        m_lampLit[i] = int(on);
        changed = true;
        const unsigned char* c = on ? lit[L.slot] : dark[L.slot];
        for (int k = 0; k < m_lampCells[i].second; ++k)
            colors->SetTypedTuple(m_lampCells[i].first + k, c);
    }
    if (changed)
    {
        colors->Modified();
        m_lampPoly->Modified();
    }
}

void MainWindow::onImpact()
{
    const auto& im = m_sim.lastImpact;
    m_lastImpactTime = m_sim.time;
    // shake: about 4.5 mm per g of the crash pulse (40 g frontal test -> 18 cm)
    if (im.peakG > 2.0)
    {
        m_shakeTime = 0.0;
        m_shakeAmp = std::clamp(0.0045 * im.peakG, 0.02, 0.45);
    }
    if (im.carKJ > 0.5)
        spawnDebris();
    showCrashReport();
    if (m_sim.impacts == 1 && !m_sim.crash.happened)
        m_crashDock->show();
    statusBar()->showMessage(
        QString(ko(u8"💥 %1와(과) 충돌 — 시속 %2 km, %3 g, 손상률 %4% (100%면 폐차)%5"))
            .arg(ko(World::kindNameKo(im.kind))).arg(im.speedKmh, 0, 'f', 0).arg(im.peakG, 0, 'f', 0)
            .arg(m_sim.damagePct, 0, 'f', 0)
            .arg(im.knocked ? ko(u8" · 물체가 튕겨 나감") : QString()),
        6000);
}

void MainWindow::updateDamageView()
{
    // strain colors: for 4 s after an impact, always after the wreck, or toggled with K
    const bool want = m_sim.impacts > 0 &&
                      (m_sim.crash.happened || m_strainToggle || m_sim.time - m_lastImpactTime < 4.0);
    if (want != m_damageShown || (want && m_sim.deformer.animating()))
    {
        showDamage(want);
        m_damageShown = want;
    }
}

void MainWindow::startCrashAnimation()
{
    // wrecked: orbit around the car, show the final report
    m_crashAnimTime = 0.0;
    m_crashOrbit = 0.0;
    m_crashHeading = m_sim.vehicle.heading() - kPi;
    showDamage(true);
    m_damageShown = true;
    showCrashReport();
    m_crashDock->show();
    m_crashDock->raise();
}

void MainWindow::finishCrashAnimation()
{
    m_crashAnimTime = -1.0;
}

void MainWindow::showDamage(bool on)
{
    // strain scale: at least 0..12 % so that a scratch stays blue-ish and a wreck reaches red
    // above 50 % the sheet metal is torn / folded flat: shown in the top band
    const double maxV = std::clamp(std::max(m_sim.deformer.maxStrainPct(), m_sim.deformer.finalStrainPct()), 12.0, 50.0);
    m_damageLut->SetTableRange(0.0, maxV);
    for (int pid : { World::PidCab, World::PidCargo })
        m_pidActorManager->setDamageView(pid, on, m_damageLut, maxV);
    m_scalarBar->SetVisibility(on);
}

// =====================================================================
// slots
// =====================================================================
void MainWindow::startMotion()
{
    if (!m_motionTimer->isActive())
    {
        m_frameClock.invalidate();
        m_simAccum = 0.0;
        // 1 ms: effectively "every display refresh", since Render() waits for vsync
        m_motionTimer->start(1);
    }
    m_vtkWidget->setFocus();
}

void MainWindow::stopMotion()
{
    if (m_motionTimer->isActive())
        m_motionTimer->stop();
    updateHud();
    m_renderWindow->Render();
}

void MainWindow::resetMotion()
{
    const bool wasRunning = m_motionTimer->isActive();
    m_motionTimer->stop();

    // keep this run as the ghost for the next one
    if (m_sim.path.size() > 50)
        m_ghostPath = m_sim.path;
    m_ghostIndex = 0;

    m_sim.reset();
    showDamage(false);
    m_damageShown = false;
    clearCrashReport();   // new run: no damage, no grade
    m_lastImpactTime = -100.0;
    m_seenImpact = m_sim.impactSerial;
    clearDebris();
    m_crashAnimTime = -1.0;
    m_cameraInitialized = false;
    m_keys.clear();

    refreshAll();

    statusBar()->showMessage(m_ghostPath.empty() ? ko(u8"리셋")
                                                 : ko(u8"리셋 — 방금 주행이 하늘색 고스트로 재생됩니다"), 5000);
    if (wasRunning)
        startMotion();
}

void MainWindow::setAutopilot(bool on)
{
    m_sim.pilot.reset();
    if (on)
    {
        double s, lat;
        m_sim.egoRoadPosition(s, lat);
        m_sim.pilot.setTargetLane(m_sim.world.road().laneOf(lat));
        startMotion();
    }
    updateHud();
    m_vtkWidget->setFocus();
}

void MainWindow::setCameraMode(int mode)
{
    // Free mode keeps the current frame fixed in the world (no jump when switching)
    if (mode == CamFree)
        m_freeFrame = m_lastFrame;

    // entering / leaving the driver's seat: start looking straight ahead
    const bool first = (mode == CamFirst);
    if (first != (m_cameraMode == CamFirst))
    {
        m_userRot = Quat::identity();
        m_zoom = 1.0;
    }
    if (!first)
        m_lastThirdPersonMode = mode;
    setFirstPersonLook(first);

    m_cameraMode = mode;
    m_cameraInitialized = false;
    updateMainCamera(0.0);
    updateHud();
    m_renderWindow->Render();
    m_vtkWidget->setFocus();
}

void MainWindow::applyVisualToggles()
{
    if (!m_renderWindow)
        return;

    const bool lidar = m_lidarAction->isChecked();
    m_cloudActor->SetVisibility(lidar);
    m_obstacleCloudActor->SetVisibility(lidar);
    m_lidarViewCloudActor->SetVisibility(lidar);
    m_lidarViewObstacleActor->SetVisibility(lidar);

    const bool sky = m_skyAction->isChecked();
    m_skybox->SetVisibility(sky);
    m_renderer->SetGradientBackground(!sky);
    m_renderer->SetBackground(0.55, 0.60, 0.66);
    m_renderer->SetBackground2(0.12, 0.18, 0.30);

    const bool shadows = m_shadowAction->isChecked();
    for (auto& a : m_shadowActors)
        a->SetVisibility(shadows);

    // PBR + image based lighting from the procedural sky: metallic / roughness per material group
    const bool pbr = m_pbrAction->isChecked();
    m_renderer->SetUseImageBasedLighting(pbr);
    m_renderer->SetEnvironmentTexture(pbr ? m_iblTexture.GetPointer() : nullptr, true);
    // Only reflective surfaces use PBR: the PBR shader costs ~3x per pixel and large matte areas
    // (road, terrain, trees, walls) look the same with classic Phong shading.
    struct Mat { int pid; double metallic, roughness; };
    const Mat mats[] = {
        { World::PidCab, 0.45, 0.32 },  { World::PidCargo, 0.35, 0.4 },  { World::PidGlass, 0.85, 0.08 } };
    for (const auto& m : mats)
        m_pidActorManager->setPbr(m.pid, pbr, m.metallic, m.roughness);
    for (auto& npc : m_npcActors)
    {
        vtkProperty* p = npc->GetProperty();
        if (pbr)
        {
            p->SetInterpolationToPBR();
            p->SetMetallic(0.5);
            p->SetRoughness(0.3);
            p->SetCoatStrength(1.0);
            p->SetCoatRoughness(0.05);
        }
        else
        {
            p->SetInterpolationToGouraud();
        }
    }
    {
        vtkProperty* p = m_waterActor->GetProperty();
        if (pbr)
        {
            p->SetInterpolationToPBR();
            p->SetMetallic(0.0);
            p->SetRoughness(0.04);
        }
        else
        {
            p->SetInterpolationToGouraud();
        }
    }

    const bool edges = m_edgesAction->isChecked();
    m_pidActorManager->setEdgesVisible(edges);
    if (m_texAsphalt)
        applyTextures(m_textureAction->isChecked());
    for (auto& npc : m_npcActors)
        npc->GetProperty()->SetEdgeVisibility(edges);

    // high quality: SSAO + FXAA + fog / surface detail (not together with the mesh edges, which use
    // their own shader path)
    const bool hq = m_hqAction->isChecked();
    m_renderer->SetPass(hq ? m_ssaoPass.GetPointer() : nullptr);
    m_renderer->SetUseFXAA(hq);
    const bool atm = hq && !edges;
    for (int pid : m_pidActorManager->getPidList())
        if (vtkActor* a = m_pidActorManager->getActorByPid(pid))
            // the tunnel is lit by its own lamps, not the sun: mostly ambient
            a->GetProperty()->SetAmbient(World::isVehiclePid(pid) ? 0.12 : (pid == World::PidTunnel ? 0.42 : 0.22));
    for (auto& npc : m_npcActors)
        npc->GetProperty()->SetAmbient(0.12);
    for (int pid : m_pidActorManager->getPidList())
    {
        const bool vehicle = World::isVehiclePid(pid);
        const double detail = vehicle || pid == World::PidGlass ? 0.0
                            : (pid == World::PidRoad ? 0.08 : (pid == World::PidTrees ? 0.2 : 0.12));
        if (vtkActor* a = m_pidActorManager->getActorByPid(pid))
            setAtmosphere(a, detail, atm);
    }
    for (auto& npc : m_npcActors)
        setAtmosphere(npc, 0.0, atm);
    setAtmosphere(m_terrainActor, 0.25, atm);
    setAtmosphere(m_waterActor, 0.0, atm);
    for (auto& a : m_shadowActors)
        setAtmosphere(a, 0.0, atm);
    for (auto& [pid, a] : m_frontCamActors)
        setAtmosphere(a, World::isVehiclePid(pid) ? 0.0 : 0.12, atm);

    updateGhost();
    m_renderWindow->Render();
    if (m_vtkWidget)
        m_vtkWidget->setFocus();
}

void MainWindow::setSurfaceMode()
{
    m_pidActorManager->setSurfaceMode();
    m_renderWindow->Render();
}

void MainWindow::setWireframeMode()
{
    m_pidActorManager->setWireframeMode();
    m_renderWindow->Render();
}

void MainWindow::onTreeItemChanged(QTreeWidgetItem* item, int column)
{
    Q_UNUSED(column);
    if (!item)
        return;

    const int pid = item->data(0, Qt::UserRole).toInt();
    const bool visible = (item->checkState(0) == Qt::Checked);

    m_pidActorManager->setVisibility(pid, visible);
    updateFrontCamera();
    m_renderWindow->Render();

    statusBar()->showMessage(QString("PID %1 ").arg(pid) + ko(World::pidNameKo(pid)) +
                             (visible ? ko(u8" 보이기") : ko(u8" 숨기기")), 3000);
}

void MainWindow::onCurrentTreeItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* previous)
{
    Q_UNUSED(previous);
    if (!current)
    {
        m_pidActorManager->clearSelection();
        m_renderWindow->Render();
        return;
    }

    const int pid = current->data(0, Qt::UserRole).toInt();
    m_pidActorManager->setSelectedPid(pid);
    m_renderWindow->Render();
    statusBar()->showMessage(QString("PID %1 ").arg(pid) + ko(World::pidNameKo(pid)) + ko(u8" 선택 (노란 선)"), 3000);
}

// =====================================================================
// picking (original example)
// =====================================================================
void MainWindow::handlePick(int displayX, int displayY)
{
    if (!m_cellPicker)
        return;

    m_cellPicker->Pick(displayX, displayY, 0.0, m_renderer);

    vtkActor* pickedActor = m_cellPicker->GetActor();
    const vtkIdType pickedCellId = m_cellPicker->GetCellId();

    if (!pickedActor || pickedCellId < 0)
    {
        m_pidActorManager->clearSelection();
        m_renderWindow->Render();
        return;
    }

    const int pid = m_pidActorManager->findPidByActor(pickedActor);
    if (pid < 0)
    {
        m_pidActorManager->clearSelection();
        m_renderWindow->Render();
        return;
    }

    m_pidActorManager->setSelectedPid(pid);
    selectTreeItemByPid(pid);
    m_renderWindow->Render();

    QString msg = QString("PID %1 ").arg(pid) + ko(World::pidNameKo(pid)) +
                  QString(", cell %1").arg(static_cast<long long>(pickedCellId));
    if (m_sim.crash.happened && (pid == World::PidCab || pid == World::PidCargo))
    {
        vtkUnstructuredGrid* g = m_sim.world.grid(pid);
        if (auto* arr = g->GetCellData()->GetArray(CrashDeformer::kArrayName))
            msg += ko(u8", 변형량 ") + QString("%1 cm").arg(100.0 * arr->GetTuple1(pickedCellId), 0, 'f', 1);
        if (auto* arr = g->GetCellData()->GetArray(CrashDeformer::kStrainArray))
            msg += ko(u8", 변형률 ") + QString("%1 %").arg(arr->GetTuple1(pickedCellId), 0, 'f', 1);
    }
    statusBar()->showMessage(msg, 6000);
}

void MainWindow::selectTreeItemByPid(int pid)
{
    for (int i = 0; i < m_treeWidget->topLevelItemCount(); ++i)
    {
        QTreeWidgetItem* item = m_treeWidget->topLevelItem(i);
        if (item && item->data(0, Qt::UserRole).toInt() == pid)
        {
            m_treeWidget->setCurrentItem(item);
            return;
        }
    }
}

// =====================================================================
// keyboard
// =====================================================================
bool MainWindow::eventFilter(QObject* watched, QEvent* event)
{
    if (event->type() == QEvent::FocusOut && watched == m_vtkWidget)
        m_keys.clear();

    // ---- mouse on the 3D view: quaternion arcball camera, click = pick ----
    if (watched == m_vtkWidget)
    {
        switch (event->type())
        {
        case QEvent::MouseButtonPress:
        {
            auto* me = static_cast<QMouseEvent*>(event);
            if (me->button() == Qt::LeftButton || me->button() == Qt::RightButton)
            {
                m_dragging = true;
                m_dragMoved = false;
                m_lastMouse = m_pressMouse = me->position().toPoint();
                m_vtkWidget->setFocus();
            }
            return true;
        }
        case QEvent::MouseMove:
        {
            auto* me = static_cast<QMouseEvent*>(event);
            if (m_dragging)
            {
                const QPoint p = me->position().toPoint();
                if ((p - m_pressMouse).manhattanLength() > 3)
                    m_dragMoved = true;
                if (m_dragMoved)
                    arcballDrag(m_lastMouse, p);
                m_lastMouse = p;
            }
            return true;
        }
        case QEvent::MouseButtonRelease:
        {
            auto* me = static_cast<QMouseEvent*>(event);
            if (m_dragging && !m_dragMoved && me->button() == Qt::LeftButton)
                pickAt(me->position().toPoint());
            m_dragging = false;
            m_idleClock.restart();
            return true;
        }
        case QEvent::MouseButtonDblClick:
            resetView();
            return true;
        case QEvent::Wheel:
        {
            auto* we = static_cast<QWheelEvent*>(event);
            m_zoom = std::clamp(m_zoom * std::pow(0.88, we->angleDelta().y() / 120.0), 0.25, 6.0);
            m_idleClock.restart();
            if (!m_motionTimer->isActive())
            {
                updateMainCamera(0.0);
                m_renderWindow->Render();
            }
            return true;
        }
        default:
            break;
        }
    }

    if (event->type() != QEvent::KeyPress && event->type() != QEvent::KeyRelease)
        return QMainWindow::eventFilter(watched, event);

    auto* ke = static_cast<QKeyEvent*>(event);
    const int key = ke->key();
    const bool driveKey = key == Qt::Key_W || key == Qt::Key_A || key == Qt::Key_S || key == Qt::Key_D ||
                          key == Qt::Key_Up || key == Qt::Key_Down || key == Qt::Key_Left ||
                          key == Qt::Key_Right || key == Qt::Key_Space;

    if (event->type() == QEvent::KeyRelease)
    {
        if (!ke->isAutoRepeat())
            m_keys.remove(key);
        return true;
    }

    if (ke->isAutoRepeat())
        return true;

    if (driveKey)
    {
        m_keys.insert(key);
        if (!m_motionTimer->isActive() && !m_sim.crash.happened)
            startMotion();
        return true;
    }

    switch (key)
    {
    case Qt::Key_T: m_autopilotAction->toggle(); break;
    case Qt::Key_X:   // driver aids (ABS / TCS / ESC) on / off: off = the car can drift
        m_sim.userDriverAids = !m_sim.userDriverAids;
        statusBar()->showMessage(m_sim.userDriverAids
                                     ? ko(u8"주행 보조(ABS·TCS·ESC) 켜짐")
                                     : ko(u8"주행 보조 꺼짐 — 풀 가속·Space(사이드 브레이크)로 드리프트 가능"), 4000);
        updateHud();
        break;
    case Qt::Key_E:   // upshift (paddle +): switches to manual mode
    case Qt::Key_Q:   // downshift (paddle -)
        if (m_sim.autopilotEnabled)
        {
            statusBar()->showMessage(ko(u8"자율주행 중에는 자동 변속(D)만 씁니다"), 3000);
            break;
        }
        m_sim.vehicle.manualShift = true;
        if (key == Qt::Key_E) m_sim.vehicle.shiftUp(); else m_sim.vehicle.shiftDown();
        statusBar()->showMessage(QString(ko(u8"수동 변속 M%1  (M: 자동 변속으로)")).arg(m_sim.vehicle.gear()), 2500);
        break;
    case Qt::Key_K:   // damage (strain) colors on / off
        m_strainToggle = !m_strainToggle;
        statusBar()->showMessage(m_strainToggle ? ko(u8"변형률 색 계속 보기") : ko(u8"변형률 색: 충돌 직후 4초만"), 3000);
        break;
    case Qt::Key_1:
    case Qt::Key_2:
    case Qt::Key_3:
    case Qt::Key_4:
        m_vehicleCombo->setCurrentIndex(key - Qt::Key_1);
        break;
    case Qt::Key_M:
        m_sim.vehicle.manualShift = !m_sim.vehicle.manualShift && !m_sim.autopilotEnabled;
        statusBar()->showMessage(m_sim.vehicle.manualShift ? ko(u8"수동 변속 (Q: 내림, E: 올림)")
                                                           : ko(u8"자동 변속 (D)"), 3000);
        break;
    case Qt::Key_C: m_cameraCombo->setCurrentIndex((m_cameraCombo->currentIndex() + 1) % m_cameraCombo->count()); break;
    case Qt::Key_L: m_lidarAction->toggle(); break;
    case Qt::Key_G: m_ghostAction->toggle(); break;
    case Qt::Key_R: resetMotion(); break;
    case Qt::Key_V:   // game-style 1st / 3rd person toggle
        m_cameraCombo->setCurrentIndex(m_cameraMode == CamFirst ? m_lastThirdPersonMode : int(CamFirst));
        break;
    case Qt::Key_Home: resetView(); break;
    case Qt::Key_Escape:   // clear the PID selection (yellow outline)
        m_treeWidget->setCurrentItem(nullptr);
        m_pidActorManager->clearSelection();
        m_renderWindow->Render();
        break;
    case Qt::Key_F1: m_helpAction->toggle(); break;
    case Qt::Key_F3:   // technical HUD lines
        m_hudDebug = !m_hudDebug;
        updateHud();
        m_renderWindow->Render();
        break;
    case Qt::Key_F9: m_recordAction->toggle(); break;
    case Qt::Key_F12:
    {
        const QString dir = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);
        saveScreenshot(dir + "/LidarDrive_" +
                       QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + ".png");
        break;
    }
    case Qt::Key_Return:
    case Qt::Key_Enter:
        if (m_motionTimer->isActive()) stopMotion(); else startMotion();
        break;
    default:
        // swallow everything else: VTK's default keys ('q'/'e' quit the app, 'w'/'s' wireframe)
        break;
    }
    return true;
}


// =====================================================================
// impact effects: flying fragments + camera shake
// =====================================================================
void MainWindow::setupDebris()
{
    m_debrisPoly = vtkSmartPointer<vtkPolyData>::New();
    vtkNew<vtkPoints> pts;
    m_debrisPoly->SetPoints(pts);
    vtkNew<vtkDoubleArray> dirs;
    dirs->SetName("orient");
    dirs->SetNumberOfComponents(3);
    m_debrisPoly->GetPointData()->SetVectors(dirs);
    vtkNew<vtkUnsignedCharArray> rgb;
    rgb->SetName("rgb");
    rgb->SetNumberOfComponents(3);
    m_debrisPoly->GetPointData()->SetScalars(rgb);

    // a fragment: thin shard of glass / paint / plastic
    vtkNew<vtkCubeSource> shard;
    shard->SetXLength(0.09);
    shard->SetYLength(0.055);
    shard->SetZLength(0.012);

    vtkNew<vtkGlyph3D> glyph;
    glyph->SetInputData(m_debrisPoly);
    glyph->SetSourceConnection(shard->GetOutputPort());
    glyph->OrientOn();
    glyph->SetVectorModeToUseVector();
    glyph->ScalingOff();
    glyph->SetColorModeToColorByScalar();

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(glyph->GetOutputPort());
    mapper->SetColorModeToDirectScalars();
    mapper->SetScalarModeToUsePointData();

    m_debrisActor = vtkSmartPointer<vtkActor>::New();
    m_debrisActor->SetMapper(mapper);
    m_debrisActor->GetProperty()->SetSpecular(0.6);
    m_debrisActor->GetProperty()->SetSpecularPower(40.0);
    m_debrisActor->PickableOff();
    m_debrisActor->SetVisibility(0);
    m_renderer->AddActor(m_debrisActor);
}

void MainWindow::clearDebris()
{
    m_debris.clear();
    m_shakeTime = -1.0;
    if (m_debrisActor)
        m_debrisActor->SetVisibility(0);
}

void MainWindow::spawnDebris()
{
    const auto& r = m_sim.report;
    const auto& cr = m_sim.crash;
    const int count = std::clamp(static_cast<int>(r.effectiveKmh * 3.5), 6, 320);
    const double ve = cr.speed * r.severity;

    double nW[3];
    m_sim.vehicle.localDirToWorld(cr.normalLocal, nW);   // into the vehicle
    const double side[3] = { -nW[1], nW[0], 0.0 };

    std::mt19937 rng(static_cast<unsigned>(m_sim.time * 1000.0) + 7u);
    std::uniform_real_distribution<double> U(0.0, 1.0);

    static const unsigned char glass[3] = { 185, 215, 230 }, trim[3] = { 30, 30, 34 };
    const int paintIdx = m_sim.vehicleType() == VehicleShapes::TypeSedan ? int(Palette::EgoSilver)
                       : m_sim.vehicleType() == VehicleShapes::TypeSuv   ? int(Palette::EgoBlack)
                                                                         : int(Palette::Body);
    const double* pc = Palette::color(paintIdx);
    const unsigned char paint[3] = { static_cast<unsigned char>(255 * pc[0]), static_cast<unsigned char>(255 * pc[1]),
                                     static_cast<unsigned char>(255 * pc[2]) };

    m_debris.clear();
    vtkPoints* pts = m_debrisPoly->GetPoints();
    auto* rgb = vtkUnsignedCharArray::SafeDownCast(m_debrisPoly->GetPointData()->GetScalars());
    auto* dirs = vtkDoubleArray::SafeDownCast(m_debrisPoly->GetPointData()->GetVectors());
    pts->SetNumberOfPoints(count);
    rgb->SetNumberOfTuples(count);
    dirs->SetNumberOfTuples(count);
    for (int i = 0; i < count; ++i)
    {
        Debris d{};
        const double s = U(rng) * 2.0 - 1.0;
        for (int k = 0; k < 3; ++k)
            d.p[k] = cr.contactWorld[k] + 0.6 * s * side[k] + 0.25 * nW[k];
        d.p[2] = m_sim.world.road().groundZ + 0.3 + 0.8 * U(rng);
        // fragments burst sideways and up, a few bounce back toward the vehicle
        const double sp = 1.0 + (0.12 + 0.35 * U(rng)) * ve;
        const double lateral = (U(rng) < 0.5 ? -1.0 : 1.0) * (0.4 + 0.8 * U(rng));
        const double back = -0.3 + 0.9 * U(rng);
        double dir[3] = { lateral * side[0] + back * nW[0], lateral * side[1] + back * nW[1], 0.3 + 0.9 * U(rng) };
        const double dl = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
        for (int k = 0; k < 3; ++k) d.v[k] = sp * dir[k] / dl;
        d.spin = 6.0 + 20.0 * U(rng);
        d.phase = 6.283 * U(rng);
        d.tilt = U(rng) * 2.0 - 1.0;
        d.rest = false;
        m_debris.push_back(d);

        const double c = U(rng);
        const unsigned char* col = c < 0.45 ? glass : c < 0.8 ? paint : trim;
        rgb->SetTypedTuple(i, col);
    }
    rgb->Modified();
    m_debrisActor->SetVisibility(1);
    updateDebris(0.0);
}

void MainWindow::updateDebris(double dt)
{
    if (m_debris.empty())
        return;
    const double ground = m_sim.world.road().groundZ + 0.008;
    vtkPoints* pts = m_debrisPoly->GetPoints();
    auto* dirs = vtkDoubleArray::SafeDownCast(m_debrisPoly->GetPointData()->GetVectors());
    for (size_t i = 0; i < m_debris.size(); ++i)
    {
        Debris& d = m_debris[i];
        if (!d.rest && dt > 0.0)
        {
            d.v[2] -= 9.81 * dt;
            for (int k = 0; k < 3; ++k)
            {
                d.v[k] *= std::exp(-0.25 * dt);   // air drag
                d.p[k] += d.v[k] * dt;
            }
            if (d.p[2] < ground)
            {
                // bounce with heavy losses, then slide to rest
                d.p[2] = ground;
                d.v[2] = -0.3 * d.v[2];
                d.v[0] *= 0.55;
                d.v[1] *= 0.55;
                d.spin *= 0.5;
                if (std::abs(d.v[2]) < 0.5 && std::hypot(d.v[0], d.v[1]) < 0.4)
                    d.rest = true;
            }
            d.phase += d.spin * dt;
        }
        pts->SetPoint(static_cast<vtkIdType>(i), d.p);
        // tumbling orientation; lying flat (random yaw) once it rests
        const double o[3] = { std::cos(d.phase), std::sin(d.phase), d.rest ? 0.0 : d.tilt * std::sin(1.7 * d.phase) };
        dirs->SetTuple(static_cast<vtkIdType>(i), o);
    }
    pts->Modified();
    dirs->Modified();
    m_debrisPoly->Modified();
}

void MainWindow::applyCameraShake()
{
    if (m_shakeTime < 0.0)
        return;
    const double t = m_shakeTime;
    const double amp = m_shakeAmp * std::exp(-4.5 * t);
    if (amp < 0.002)
    {
        m_shakeTime = -1.0;
        return;
    }
    // sum of incommensurate sines: looks random, no visible pattern
    const double off[3] = { amp * (std::sin(t * 61.0 + 0.3) + 0.5 * std::sin(t * 103.0 + 1.1)),
                            amp * (std::sin(t * 53.0 + 2.0) + 0.5 * std::sin(t * 89.0 + 0.7)),
                            amp * 0.7 * (std::sin(t * 71.0 + 1.4) + 0.5 * std::sin(t * 127.0)) };
    vtkCamera* cam = m_renderer->GetActiveCamera();
    double p[3], f[3];
    cam->GetPosition(p);
    cam->GetFocalPoint(f);
    for (int k = 0; k < 3; ++k)
    {
        p[k] += off[k];
        f[k] += 0.4 * off[k];
    }
    cam->SetPosition(p);
    cam->SetFocalPoint(f);
}

// =====================================================================
// Game-style instrument cluster (VTK 2D actors, rebuilt every frame in display pixels):
// speedometer + tachometer arcs with ticks, digital speed, gear, damage bar; screen vignette
// =====================================================================
namespace
{
// filled ring segment from angle a0 to a1 (deg, counter-clockwise), radius r0..r1, colored per quad
void addArc(vtkPoints* pts, vtkCellArray* polys, vtkUnsignedCharArray* rgba, double cx, double cy, double r0,
            double r1, double a0, double a1, const unsigned char c0[4], const unsigned char c1[4], int segs)
{
    for (int i = 0; i < segs; ++i)
    {
        const double t0 = static_cast<double>(i) / segs, t1 = static_cast<double>(i + 1) / segs;
        const double b0 = (a0 + (a1 - a0) * t0) * 3.14159265358979323846 / 180.0;
        const double b1 = (a0 + (a1 - a0) * t1) * 3.14159265358979323846 / 180.0;
        const vtkIdType p0 = pts->InsertNextPoint(cx + r0 * std::cos(b0), cy + r0 * std::sin(b0), 0.0);
        const vtkIdType p1 = pts->InsertNextPoint(cx + r1 * std::cos(b0), cy + r1 * std::sin(b0), 0.0);
        const vtkIdType p2 = pts->InsertNextPoint(cx + r1 * std::cos(b1), cy + r1 * std::sin(b1), 0.0);
        const vtkIdType p3 = pts->InsertNextPoint(cx + r0 * std::cos(b1), cy + r0 * std::sin(b1), 0.0);
        const vtkIdType q[4] = { p0, p1, p2, p3 };
        polys->InsertNextCell(4, q);
        unsigned char c[4];
        for (int k = 0; k < 4; ++k)
            c[k] = static_cast<unsigned char>(c0[k] + (c1[k] - c0[k]) * (0.5 * (t0 + t1)));
        for (int k = 0; k < 4; ++k)
            rgba->InsertNextTypedTuple(c);
    }
}

void addRect(vtkPoints* pts, vtkCellArray* polys, vtkUnsignedCharArray* rgba, double x0, double y0, double x1,
             double y1, const unsigned char c[4])
{
    const vtkIdType a = pts->InsertNextPoint(x0, y0, 0.0), b = pts->InsertNextPoint(x1, y0, 0.0);
    const vtkIdType d = pts->InsertNextPoint(x1, y1, 0.0), e = pts->InsertNextPoint(x0, y1, 0.0);
    const vtkIdType q[4] = { a, b, d, e };
    polys->InsertNextCell(4, q);
    for (int k = 0; k < 4; ++k)
        rgba->InsertNextTypedTuple(c);
}

vtkSmartPointer<vtkTextActor> makeGaugeText(int size, bool bold, double r, double g, double b)
{
    auto t = vtkSmartPointer<vtkTextActor>::New();
    t->GetPositionCoordinate()->SetCoordinateSystemToDisplay();
    vtkTextProperty* p = t->GetTextProperty();
    p->SetFontFamilyToArial();
    p->SetFontSize(size);
    p->SetBold(bold);
    p->SetColor(r, g, b);
    p->SetJustificationToCentered();
    p->SetVerticalJustificationToCentered();
    p->SetShadow(true);
    p->SetShadowOffset(1, -1);
    return t;
}
}

void MainWindow::setupGauges()
{
    m_gaugePoly = vtkSmartPointer<vtkPolyData>::New();
    vtkNew<vtkPolyDataMapper2D> mapper;
    mapper->SetInputData(m_gaugePoly);
    mapper->SetColorModeToDirectScalars();
    mapper->SetScalarModeToUsePointData();
    m_gaugeActor = vtkSmartPointer<vtkActor2D>::New();
    m_gaugeActor->SetMapper(mapper);

    m_vignettePoly = vtkSmartPointer<vtkPolyData>::New();
    vtkNew<vtkPolyDataMapper2D> vm;
    vm->SetInputData(m_vignettePoly);
    vm->SetColorModeToDirectScalars();
    vm->SetScalarModeToUsePointData();
    m_vignetteActor = vtkSmartPointer<vtkActor2D>::New();
    m_vignetteActor->SetMapper(vm);
    // the vignette goes behind every other HUD element
    m_hudRenderer->AddActor2D(m_vignetteActor);
    m_hudRenderer->AddActor2D(m_gaugeActor);

    m_speedText = makeGaugeText(40, true, 1.0, 1.0, 1.0);
    m_speedUnitText = makeGaugeText(13, false, 0.7, 0.8, 0.9);
    m_gearText = makeGaugeText(34, true, 1.0, 0.75, 0.25);
    m_rpmText = makeGaugeText(12, false, 0.7, 0.8, 0.9);
    m_damageText = makeGaugeText(12, true, 1.0, 1.0, 1.0);
    for (auto* t : { m_speedText.GetPointer(), m_speedUnitText.GetPointer(), m_gearText.GetPointer(),
                     m_rpmText.GetPointer(), m_damageText.GetPointer() })
        m_hudRenderer->AddActor2D(t);
}

void MainWindow::updateGauges()
{
    const int* sz = m_renderWindow->GetSize();
    if (!m_gaugePoly || sz[0] <= 0 || sz[1] <= 0)
        return;
    const double W = sz[0], H = sz[1];
    const VehicleModel& v = m_sim.vehicle;

    vtkNew<vtkPoints> pts;
    vtkNew<vtkCellArray> polys;
    vtkNew<vtkUnsignedCharArray> rgba;
    rgba->SetNumberOfComponents(4);

    // ---- vignette: darker toward the edges (two soft frames)
    {
        vtkNew<vtkPoints> vp;
        vtkNew<vtkCellArray> vc;
        vtkNew<vtkUnsignedCharArray> va;
        va->SetNumberOfComponents(4);
        const double inset[3] = { 0.0, 0.10, 0.24 };
        const unsigned char alpha[3] = { 120, 35, 0 };
        for (int ring = 0; ring < 2; ++ring)
        {
            const double o = inset[ring], in = inset[ring + 1];
            const double ox0 = o * W * 0.5, oy0 = o * H * 0.5, ox1 = W - ox0, oy1 = H - oy0;
            const double ix0 = in * W * 0.5, iy0 = in * H * 0.5, ix1 = W - ix0, iy1 = H - iy0;
            const double outer[4][2] = { { ox0, oy0 }, { ox1, oy0 }, { ox1, oy1 }, { ox0, oy1 } };
            const double inner[4][2] = { { ix0, iy0 }, { ix1, iy0 }, { ix1, iy1 }, { ix0, iy1 } };
            for (int k = 0; k < 4; ++k)
            {
                const int k2 = (k + 1) % 4;
                const vtkIdType a = vp->InsertNextPoint(outer[k][0], outer[k][1], 0.0);
                const vtkIdType b = vp->InsertNextPoint(outer[k2][0], outer[k2][1], 0.0);
                const vtkIdType c = vp->InsertNextPoint(inner[k2][0], inner[k2][1], 0.0);
                const vtkIdType d = vp->InsertNextPoint(inner[k][0], inner[k][1], 0.0);
                const vtkIdType q[4] = { a, b, c, d };
                vc->InsertNextCell(4, q);
                const unsigned char co[4] = { 0, 0, 0, alpha[ring] }, ci[4] = { 0, 0, 0, alpha[ring + 1] };
                va->InsertNextTypedTuple(co);
                va->InsertNextTypedTuple(co);
                va->InsertNextTypedTuple(ci);
                va->InsertNextTypedTuple(ci);
            }
        }
        m_vignettePoly->SetPoints(vp);
        m_vignettePoly->SetPolys(vc);
        m_vignettePoly->GetPointData()->SetScalars(va);
        m_vignetteActor->SetVisibility(m_hqAction->isChecked());
    }

    // ---- two dials at the bottom center
    const double R = std::clamp(0.105 * H, 62.0, 105.0);
    const double cyD = R + 0.035 * H;
    const double cxS = 0.5 * W - 1.12 * R, cxT = 0.5 * W + 1.12 * R;
    const double a0 = 225.0, sweep = 270.0;   // from bottom-left, clockwise to bottom-right
    const unsigned char bg[4] = { 8, 12, 20, 165 }, track[4] = { 255, 255, 255, 38 };
    // dark discs
    for (double cx : { cxS, cxT })
        addArc(pts, polys, rgba, cx, cyD, 0.0, R, 0.0, 360.0, bg, bg, 48);
    // speed
    const double vMax = std::ceil(v.maxSpeed * 3.6 / 20.0) * 20.0;
    const double kmh = std::abs(v.speed()) * 3.6;
    const double fs = std::clamp(kmh / vMax, 0.0, 1.0);
    addArc(pts, polys, rgba, cxS, cyD, 0.78 * R, 0.90 * R, a0, a0 - sweep, track, track, 54);
    {
        const unsigned char c0[4] = { 40, 200, 255, 255 }, c1[4] = { 120, 90, 255, 255 };
        if (fs > 0.002)
            addArc(pts, polys, rgba, cxS, cyD, 0.78 * R, 0.90 * R, a0, a0 - sweep * fs, c0, c1,
                   std::max(2, static_cast<int>(54 * fs)));
    }
    // speed ticks every 20 km/h (major every 40)
    for (double k = 0.0; k <= vMax + 1e-6; k += 20.0)
    {
        const double a = (a0 - sweep * k / vMax) * kPi / 180.0;
        const bool major = std::fmod(k, 40.0) < 1e-6;
        const double r0 = (major ? 0.64 : 0.69) * R, r1 = 0.75 * R, w = major ? 1.6 : 1.0;
        const unsigned char c[4] = { 220, 230, 245, static_cast<unsigned char>(major ? 230 : 140) };
        const double ca = std::cos(a), sa = std::sin(a), px = -sa * w, py = ca * w;
        const vtkIdType p0 = pts->InsertNextPoint(cxS + r0 * ca - px, cyD + r0 * sa - py, 0.0);
        const vtkIdType p1 = pts->InsertNextPoint(cxS + r1 * ca - px, cyD + r1 * sa - py, 0.0);
        const vtkIdType p2 = pts->InsertNextPoint(cxS + r1 * ca + px, cyD + r1 * sa + py, 0.0);
        const vtkIdType p3 = pts->InsertNextPoint(cxS + r0 * ca + px, cyD + r0 * sa + py, 0.0);
        const vtkIdType q[4] = { p0, p1, p2, p3 };
        polys->InsertNextCell(4, q);
        for (int i = 0; i < 4; ++i)
            rgba->InsertNextTypedTuple(c);
    }
    // rpm with the red zone
    const double fr = std::clamp(v.rpm() / (v.redlineRpm * 1.05), 0.0, 1.0);
    const double redFrom = v.redlineRpm * 0.92 / (v.redlineRpm * 1.05);
    addArc(pts, polys, rgba, cxT, cyD, 0.78 * R, 0.90 * R, a0, a0 - sweep, track, track, 54);
    {
        const unsigned char red[4] = { 255, 50, 40, 120 };
        addArc(pts, polys, rgba, cxT, cyD, 0.92 * R, 0.96 * R, a0 - sweep * redFrom, a0 - sweep, red, red, 10);
        const unsigned char c0[4] = { 255, 190, 60, 255 }, c1[4] = { 255, 60, 40, 255 };
        if (fr > 0.002)
            addArc(pts, polys, rgba, cxT, cyD, 0.78 * R, 0.90 * R, a0, a0 - sweep * fr, c0,
                   fr > redFrom ? c1 : c0, std::max(2, static_cast<int>(54 * fr)));
    }
    for (int k = 0; k <= static_cast<int>(v.redlineRpm * 1.05 / 1000.0); ++k)
    {
        const double a = (a0 - sweep * (k * 1000.0) / (v.redlineRpm * 1.05)) * kPi / 180.0;
        const double r0 = 0.66 * R, r1 = 0.75 * R, w = 1.4;
        const unsigned char c[4] = { 220, 230, 245, 200 };
        const double ca = std::cos(a), sa = std::sin(a), px = -sa * w, py = ca * w;
        const vtkIdType p0 = pts->InsertNextPoint(cxT + r0 * ca - px, cyD + r0 * sa - py, 0.0);
        const vtkIdType p1 = pts->InsertNextPoint(cxT + r1 * ca - px, cyD + r1 * sa - py, 0.0);
        const vtkIdType p2 = pts->InsertNextPoint(cxT + r1 * ca + px, cyD + r1 * sa + py, 0.0);
        const vtkIdType p3 = pts->InsertNextPoint(cxT + r0 * ca + px, cyD + r0 * sa + py, 0.0);
        const vtkIdType q[4] = { p0, p1, p2, p3 };
        polys->InsertNextCell(4, q);
        for (int i = 0; i < 4; ++i)
            rgba->InsertNextTypedTuple(c);
    }
    // damage bar under the dials (after the first impact)
    const bool showDamage = m_sim.impacts > 0;
    if (showDamage)
    {
        const double bw = 2.0 * R, bh = 7.0, bx = 0.5 * W - 0.5 * bw, by = cyD - R - 4.0;
        const unsigned char back[4] = { 255, 255, 255, 40 };
        addRect(pts, polys, rgba, bx, by, bx + bw, by + bh, back);
        const double f = std::clamp(m_sim.damagePct / 100.0, 0.0, 1.0);
        const unsigned char col[4] = { static_cast<unsigned char>(f < 0.4 ? 80 + 400 * f : 255),
                                       static_cast<unsigned char>(f < 0.4 ? 220 : 220 - 180 * (f - 0.4) / 0.6),
                                       60, 235 };
        addRect(pts, polys, rgba, bx, by, bx + bw * f, by + bh, col);
    }
    m_gaugePoly->SetPoints(pts);
    m_gaugePoly->SetPolys(polys);
    m_gaugePoly->GetPointData()->SetScalars(rgba);
    m_gaugePoly->Modified();

    m_speedText->SetInput(QString::number(static_cast<int>(std::round(kmh))).toUtf8().constData());
    m_speedText->SetPosition(cxS, cyD + 0.04 * R);
    m_speedUnitText->SetInput("km/h");
    m_speedUnitText->SetPosition(cxS, cyD - 0.36 * R);
    const QString gear = v.reverse() ? QString("R") : (std::abs(v.speed()) < 0.2 && v.rpm() < v.idleRpm * 1.2 && v.gear() == 1)
                                                            ? QString("%1").arg(v.manualShift ? "M1" : "D")
                                                            : QString("%1%2").arg(v.manualShift ? "M" : "").arg(v.gear());
    m_gearText->SetInput(gear.toUtf8().constData());
    m_gearText->SetPosition(cxT, cyD + 0.06 * R);
    m_rpmText->SetInput(QString("%1 rpm%2").arg(v.rpm(), 0, 'f', 0).arg(v.shifting() ? "  SHIFT" : "").toUtf8().constData());
    m_rpmText->SetPosition(cxT, cyD - 0.36 * R);
    m_damageText->SetVisibility(showDamage);
    if (showDamage)
    {
        m_damageText->SetInput(QString("DAMAGE %1%").arg(m_sim.damagePct, 0, 'f', 0).toUtf8().constData());
        m_damageText->SetPosition(0.5 * W, cyD - R - 14.0);
    }
}

// =====================================================================
// Texture mapping: procedural, seamlessly tiling textures (asphalt, grass, concrete) on the road,
// the terrain and the buildings. World-space texture coordinates (planar projection), mipmaps and
// anisotropic filtering so the road stays crisp at grazing angles. The texture color MULTIPLIES the
// per-cell material color (VTK modulates the scalar colors with the texture).
// =====================================================================
namespace
{
// periodic value noise on a lattice of 'cells' x 'cells' -> tiles seamlessly
double tileNoise(double u, double v, int cells, unsigned seed)
{
    auto hash = [&](int i, int j) {
        i = ((i % cells) + cells) % cells;
        j = ((j % cells) + cells) % cells;
        unsigned h = static_cast<unsigned>(i) * 374761393u + static_cast<unsigned>(j) * 668265263u + seed * 2246822519u;
        h = (h ^ (h >> 13)) * 1274126177u;
        return ((h ^ (h >> 16)) & 0xffffff) / 16777215.0;
    };
    const double x = u * cells, y = v * cells;
    const int i = static_cast<int>(std::floor(x)), j = static_cast<int>(std::floor(y));
    double fx = x - i, fy = y - j;
    fx = fx * fx * (3 - 2 * fx);
    fy = fy * fy * (3 - 2 * fy);
    const double a = hash(i, j), b = hash(i + 1, j), c = hash(i, j + 1), d = hash(i + 1, j + 1);
    return (a + (b - a) * fx) + ((c + (d - c) * fx) - (a + (b - a) * fx)) * fy;
}

double tileFbm(double u, double v, int base, int octaves, unsigned seed)
{
    double s = 0.0, amp = 0.5, norm = 0.0;
    for (int o = 0; o < octaves; ++o)
    {
        s += amp * tileNoise(u, v, base << o, seed + o * 17u);
        norm += amp;
        amp *= 0.5;
    }
    return s / norm;
}

vtkSmartPointer<vtkTexture> makeTexture(int n, const std::function<void(double, double, double[3])>& fn)
{
    vtkNew<vtkImageData> img;
    img->SetDimensions(n, n, 1);
    img->AllocateScalars(VTK_UNSIGNED_CHAR, 3);
    auto* p = static_cast<unsigned char*>(img->GetScalarPointer());
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i)
        {
            double c[3];
            fn((i + 0.5) / n, (j + 0.5) / n, c);
            for (int k = 0; k < 3; ++k)
                *p++ = static_cast<unsigned char>(std::clamp(c[k], 0.0, 1.0) * 255.0);
        }
    auto t = vtkSmartPointer<vtkTexture>::New();
    t->SetInputData(img);
    t->RepeatOn();
    t->InterpolateOn();
    t->MipmapOn();
    t->SetMaximumAnisotropicFiltering(16.0);
    return t;
}

// planar world-space texture coordinates on every point of a dataset
template <class F>
void setTCoords(vtkDataSet* ds, F&& uv)
{
    vtkNew<vtkFloatArray> tc;
    tc->SetName("TCoords");
    tc->SetNumberOfComponents(2);
    tc->SetNumberOfTuples(ds->GetNumberOfPoints());
    for (vtkIdType i = 0; i < ds->GetNumberOfPoints(); ++i)
    {
        double p[3], t[2];
        ds->GetPoint(i, p);
        uv(p, t);
        tc->SetTuple(i, t);
    }
    ds->GetPointData()->SetTCoords(tc);
}
}

void MainWindow::setupTextures()
{
    // asphalt: aggregate speckles, darker oil stains / patches, hairline cracks (4 m per tile)
    m_texAsphalt = makeTexture(512, [](double u, double v, double c[3]) {
        const double fine = tileNoise(u, v, 256, 1);
        const double grain = tileNoise(u, v, 128, 2);
        const double blot = tileFbm(u, v, 4, 4, 3);
        double g = 0.93 + 0.10 * (grain - 0.5) + 0.12 * (fine - 0.5);
        if (fine > 0.93) g += 0.10;                      // light stones in the mix
        if (fine < 0.05) g -= 0.10;                      // dark pores
        g -= 0.14 * std::max(0.0, blot - 0.62) / 0.38;    // oil stains / patches
        const double crack = std::abs(tileFbm(u + 0.3, v, 3, 3, 9) - 0.5);
        const double crackZone = tileFbm(u, v, 2, 2, 11);
        if (crack < 0.0025 && crackZone > 0.62) g -= 0.10;   // occasional hairline cracks
        c[0] = c[1] = c[2] = g;
        c[2] += 0.01;
    });
    // grass: blades (streaks), clumps, dry tips (6 m per tile); multiplies the terrain colors
    m_texGrass = makeTexture(512, [](double u, double v, double c[3]) {
        const double clump = tileFbm(u, v, 6, 4, 21);
        const double blade = tileNoise(u * 1.0, v, 256, 22) * 0.6 + tileNoise(u, v, 96, 23) * 0.4;
        const double dry = std::max(0.0, tileFbm(u, v, 3, 3, 24) - 0.6);
        const double g = 0.90 + 0.30 * (clump - 0.5) + 0.28 * (blade - 0.5);
        c[0] = g + 0.25 * dry;
        c[1] = g + 0.08 * dry;
        c[2] = g - 0.10 + 0.05 * dry;
    });
    // concrete facade: fine noise, stains running down, a joint at every floor (3.4 m) and every 4 m
    m_texConcrete = makeTexture(512, [](double u, double v, double c[3]) {
        const double n = tileFbm(u, v, 8, 5, 31);
        const double streak = tileNoise(u, v * 0.15, 64, 32);
        double g = 0.88 + 0.12 * (n - 0.5) - 0.06 * std::max(0.0, streak - 0.6);
        const double fu = std::fmod(u * 1.0, 1.0), fv = std::fmod(v * 1.0, 1.0);
        if (fv < 0.012 || fu < 0.008)
            g -= 0.16;                                   // panel joints
        c[0] = c[1] = c[2] = g;
    });

    // texture coordinates: road (and markings) 4 m tiles, buildings: along the facade x+y / per floor z
    if (auto* g = m_sim.world.grid(World::PidRoad))
        setTCoords(g, [](const double p[3], double t[2]) { t[0] = p[0] / 4.0; t[1] = p[1] / 4.0; });
    if (auto* g = m_sim.world.grid(World::PidBuildings))
        setTCoords(g, [](const double p[3], double t[2]) { t[0] = (p[0] + p[1]) / 4.0; t[1] = (p[2] + 0.1) / 3.4; });
    if (auto* g = m_sim.world.grid(World::PidBridge))
        setTCoords(g, [](const double p[3], double t[2]) { t[0] = (p[0] + p[1]) / 4.0; t[1] = (p[2] + 0.1) / 3.4; });
    setTCoords(m_sim.world.terrain(), [](const double p[3], double t[2]) { t[0] = p[0] / 6.0; t[1] = p[1] / 6.0; });
}

void MainWindow::applyTextures(bool on)
{
    auto set = [&](vtkActor* a, vtkTexture* t) {
        if (a)
            a->SetTexture(on ? t : nullptr);
    };
    set(m_pidActorManager->getActorByPid(World::PidRoad), m_texAsphalt);
    set(m_pidActorManager->getActorByPid(World::PidBuildings), m_texConcrete);
    set(m_pidActorManager->getActorByPid(World::PidBridge), m_texConcrete);
    set(m_terrainActor, m_texGrass);
    for (const auto& [pid, a] : m_frontCamActors)
        if (pid == World::PidRoad)
            set(a, m_texAsphalt);
        else if (pid == World::PidBuildings || pid == World::PidBridge)
            set(a, m_texConcrete);
}
