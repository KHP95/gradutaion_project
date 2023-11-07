# 로봇팔🦾의 임베디드시스템 구성 프로젝트 레포입니다.
* 2020년 졸업 프로젝트로 진행하였습니다.
* 전체 프로젝트의 이름은 **행성 탐사 로버 배터리 자동 교체 시스템 개발** 입니다.
  * 로봇팔설계와 패스플랜 시뮬레이션, 임베디드시스템 구성, 머신비전까지 포함된 프로젝트입니다.
  * **저는 임베디드시스템 구성업무를 담당**하였습니다.
* 논문은 레포의 paper 확인바랍니다.

&nbsp;&nbsp;&nbsp;&nbsp;
### 세부 담당 내용 ⬇️
* 로봇팔을 제어시스템 하드웨어 구성 (구성도 참조)
* 배터리교체 알고리즘 개발 (알고리즘 참조)
* **C 기반의 로봇팔 제어 펌웨어 개발**  
  * Due로 6개의 조인트 서보 제어, Uno와 UART통신, 머신비전모듈과 UART통신
  * Uno로 2개의 기저 서보 제어, 캘리브레이션 기능수행
  * nRF모듈로 타겟 로버 시스템과 SPI무선통신
  * 미션수행, 캘리브레이션, 위치기반 조정 혹은 각도기반 조정등 각종 기능 구현

&nbsp;&nbsp;&nbsp;&nbsp;
### 구성도 ⬇️
<img src="https://github.com/KHP95/gradutaion_project/blob/main/hardware_diagram.png" width="700">
&nbsp;&nbsp;&nbsp;&nbsp;

### 알고리즘 ⬇️
<img src="https://github.com/KHP95/gradutaion_project/blob/main/algorithm_flowchart.png" width="700">
