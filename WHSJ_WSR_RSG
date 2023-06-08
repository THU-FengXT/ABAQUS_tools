# -*- coding: mbcs -*-
from part import *
from material import *
from section import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from optimization import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *

class CS():
    def __init__(self,d1,d2,d3,t1,t2,l,n1,n2,n3,dis,steel,
                 E,v,fy,fu,ey,eu,ininc,maxinc,mininc,numinc,rigid,jobname):
        self.diaOFball = float(d1)     #球节点的外径
        self.diaOFpipe = float(d2)      #相连杆件的外径
        self.diaOFplate = float(d3)      #肋板中间孔径
        self.thiOFball = float(t1)     #球节点厚度
        self.thiOFpipe = float(t2)      #相连杆件的厚度
        self.lengthOFpipe = float(l)    #相连杆件的长度
        self.numOFball = int(n1)      #球壳沿厚度方向划分n1层网格
        self.numOFpipe = int(n2)      #相连杆件沿厚度方向划分n2层网格
        self.numOFglobal = float(n3)    #全局布种密度
        self.load = float(dis)          #位移荷载
        self.steelname = steel          #材料名称
        self.Elastic =float(E)          #弹性模量
        self.Poisson =float(v)          #泊松比
        self.fy = float(fy)             #屈服强度
        self.fu = float(fu)             #抗拉强度
        self.ey = float(ey)             #强化段起始处的塑性应变
        self.eu = float(eu)             #极限状态下塑性应变
        self.ininc = float(ininc)       #初始增量步
        self.maxinc = float(maxinc)     #最大增量步长
        self.mininc = float(mininc)     #最小增量步长
        self.numinc = int(numinc)       #最大增量步数
        self.rigid = rigid              #边界约束条件是否固接
        self.jobname = jobname          #工作名称

    def part(self):
        r1 = self.diaOFball*0.5
        r2 = self.diaOFpipe*0.5
        th1 = self.thiOFball
        th2 = self.thiOFpipe
        # 建立空心球壳部件
        mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=2000.0)
        mdb.models['Model-1'].sketches['__profile__'].ConstructionLine(
            point1=(0.0,-1000.0),
            point2=(0.0, 1000.0))

        mdb.models['Model-1'].sketches['__profile__'].ArcByCenterEnds(
            center=(0.0, 0.0),
            direction=COUNTERCLOCKWISE,
            point1=(0.0, -r1),
            point2=(0.0, r1))
        mdb.models['Model-1'].sketches['__profile__'].ArcByCenterEnds(
            center=(0.0, 0.0),
            direction=COUNTERCLOCKWISE,
            point1=(0.0, -r1 + th1),
            point2=(0.0, r1 - th1))
        mdb.models['Model-1'].sketches['__profile__'].Line(
            point1=(0.0, r1),
            point2=(0.0, r1 - th1))
        mdb.models['Model-1'].sketches['__profile__'].Line(
            point1=(0.0, -r1 + th1),
            point2=(0.0, -r1))

        mdb.models['Model-1'].Part(
            dimensionality=THREE_D,
            name='ball',
            type=DEFORMABLE_BODY)
        mdb.models['Model-1'].parts['ball'].BaseSolidRevolve(
            angle=360.0,
            flipRevolveDirection=OFF,
            sketch=mdb.models['Model-1'].sketches['__profile__'])

        del mdb.models['Model-1'].sketches['__profile__']

        # 建立实心的球部件用于切割钢管

        mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=2000.0)
        mdb.models['Model-1'].sketches['__profile__'].ConstructionLine(
            point1=(0.0,-1000.0),
            point2=(0.0, 1000.0))

        mdb.models['Model-1'].sketches['__profile__'].ArcByCenterEnds(
            center=(0.0, 0.0),
            direction=COUNTERCLOCKWISE,
            point1=(0.0, -r1),
            point2=(0.0, r1))

        mdb.models['Model-1'].sketches['__profile__'].Line(
            point1=(0.0, r1),
            point2=(0.0, -r1))

        mdb.models['Model-1'].Part(
            dimensionality=THREE_D,
            name='ballCut',
            type=DEFORMABLE_BODY)

        mdb.models['Model-1'].parts['ballCut'].BaseSolidRevolve(
            angle=360.0,
            flipRevolveDirection=OFF,
            sketch=mdb.models['Model-1'].sketches['__profile__'])

        del mdb.models['Model-1'].sketches['__profile__']

        # 建立钢管部件

        mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=2000.0)

        mdb.models['Model-1'].sketches['__profile__'].CircleByCenterPerimeter(
            center=(0.0, 0.0),
            point1=(0.0, r2))

        mdb.models['Model-1'].sketches['__profile__'].CircleByCenterPerimeter(
            center=(0.0, 0.0),
            point1=(0.0, r2-th2))

        mdb.models['Model-1'].Part(
            dimensionality=THREE_D,
            name='pipe',
            type=DEFORMABLE_BODY)

        mdb.models['Model-1'].parts['pipe'].BaseSolidExtrude(
            depth=self.lengthOFpipe,
            sketch=mdb.models['Model-1'].sketches['__profile__'])

        del mdb.models['Model-1'].sketches['__profile__']

        mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=1000.0)
        mdb.models['Model-1'].sketches['__profile__'].ConstructionLine(
            point1=(0.0,-500.0),
            point2=(0.0, 500.0))
        r3 = self.diaOFplate/2
        r4 = r1 - th1
        y11 = th1/2
        x11 = (r4**2 - y11**2) ** 0.5
        mdb.models['Model-1'].sketches['__profile__'].Line(
            point1=(x11, y11),
            point2=(r3, y11))

        mdb.models['Model-1'].sketches['__profile__'].Line(
            point1=(r3, y11),
            point2=(r3, - y11))

        mdb.models['Model-1'].sketches['__profile__'].Line(
            point1=(r3, - y11),
            point2=(x11, - y11))

        mdb.models['Model-1'].sketches['__profile__'].ArcByCenterEnds(
            center=(0.0, 0.0),
            direction=COUNTERCLOCKWISE,
            point1=(x11, -y11),
            point2=(x11,  y11))

        mdb.models['Model-1'].Part(
            dimensionality=THREE_D,
            name='LeiBan',
            type=DEFORMABLE_BODY)
        mdb.models['Model-1'].parts['LeiBan'].BaseSolidRevolve(
            angle=360.0,
            flipRevolveDirection=OFF,
            sketch=mdb.models['Model-1'].sketches['__profile__'])
        del mdb.models['Model-1'].sketches['__profile__']




    def material(self):
        def ln(x):
            x = x-1
            y = x - x ** 2 / 2 + x ** 3 / 3 - x ** 4 / 4 + x ** 5 / 5
            return y
        mdb.models['Model-1'].Material(name=self.steelname)
        mdb.models['Model-1'].materials[self.steelname].Elastic(
            table=((self.Elastic, self.Poisson),))
        f1 = self.fy * (1 + self.fy/self.Elastic)
        e1 = 0
        f2 = self.fy * (1 + self.ey)
        e2 = ln(x=(1+ self.ey - self.fy/self.Elastic))
        f3 = self.fu * (1 + self.eu)
        e3 = ln(x=(1+ self.eu - self.fu/self.Elastic))
        mdb.models['Model-1'].materials[self.steelname].Plastic(
            table=((f1,e1),
            (f2, e2),
            (f3, e3)))
        mdb.models['Model-1'].HomogeneousSolidSection(
            material=self.steelname,
            name=self.steelname, thickness=None)

        mdb.models['Model-1'].parts['pipe'].Set(
            cells=mdb.models['Model-1'].parts['pipe'].cells.findAt
            ((((self.diaOFpipe - self.thiOFpipe)*0.5,0.0,0.0), )),
            name='Set-1')

        mdb.models['Model-1'].parts['pipe'].SectionAssignment(
            offset=0.0, offsetField='', offsetType=MIDDLE_SURFACE,
            region=mdb.models['Model-1'].parts['pipe'].sets['Set-1'],
            sectionName=self.steelname,thicknessAssignment=FROM_SECTION)


        mdb.models['Model-1'].parts['ball'].Set(
            cells=mdb.models['Model-1'].parts['ball'].cells.findAt
            (((0.0,0.0,(self.diaOFball - self.thiOFball)*0.5), )),
            name='Set-1')
        mdb.models['Model-1'].parts['ball'].SectionAssignment(
            offset=0.0,offsetField='', offsetType=MIDDLE_SURFACE,
            region=mdb.models['Model-1'].parts['ball'].sets['Set-1'],
            sectionName=self.steelname, thicknessAssignment=FROM_SECTION)

        mdb.models['Model-1'].parts['LeiBan'].Set(
            cells=mdb.models['Model-1'].parts['LeiBan'].cells.findAt
            (((0, 0,self.diaOFball/2 -self.thiOFball),)),
            name='Set-1')
        mdb.models['Model-1'].parts['LeiBan'].SectionAssignment(
            offset=0.0,offsetField='', offsetType=MIDDLE_SURFACE,
            region=mdb.models['Model-1'].parts['LeiBan'].sets['Set-1'],
            sectionName=self.steelname,thicknessAssignment=FROM_SECTION)

    def assembly(self):
        mdb.models['Model-1'].rootAssembly.deleteAllFeatures()
        mdb.models['Model-1'].rootAssembly.DatumCsysByDefault(CARTESIAN)

        mdb.models['Model-1'].rootAssembly.Instance(dependent=ON, name='ballCut-1',
            part=mdb.models['Model-1'].parts['ballCut'])
        mdb.models['Model-1'].rootAssembly.Instance(dependent=ON, name='pipe-1',
            part=mdb.models['Model-1'].parts['pipe'])
        #平移钢管
        mdb.models['Model-1'].rootAssembly.translate(instanceList=('pipe-1', ),
            vector= (0.0, 0.0, - self.lengthOFpipe *0.5))
        #切割钢管
        mdb.models['Model-1'].rootAssembly.InstanceFromBooleanCut(
            cuttingInstances=(mdb.models['Model-1'].rootAssembly.instances['ballCut-1'], ),
            instanceToBeCut=mdb.models['Model-1'].rootAssembly.instances['pipe-1'],
            name='pipeCut', originalInstances=SUPPRESS)
        mdb.models['Model-1'].rootAssembly.Instance(dependent=ON, name='ball-1',
            part=mdb.models['Model-1'].parts['ball'])

        #将切割后的钢管和空心球合并成一个part，命名为'Test'
        mdb.models['Model-1'].rootAssembly.InstanceFromBooleanMerge(domain=GEOMETRY,
            instances=(mdb.models['Model-1'].rootAssembly.instances['pipeCut-1'],
            mdb.models['Model-1'].rootAssembly.instances['ball-1']),
            name='Test',originalInstances=SUPPRESS)

        mdb.models['Model-1'].rootAssembly.Instance(
            dependent=ON, name='LeiBan-1',
            part=mdb.models['Model-1'].parts['LeiBan'])

    def step(self):
        mdb.models['Model-1'].StaticStep(
            initialInc=self.ininc,
            maxInc=self.maxinc,
            maxNumInc=self.numinc,
            minInc=self.mininc,
            name='load',
            nlgeom=ON,
            previous='Initial')

    def interaction(self):

        #建立第一个参考点
        mdb.models['Model-1'].rootAssembly.ReferencePoint(
            point=(0.0, 0.0, -self.lengthOFpipe * 0.5))
        for i,j in mdb.models['Model-1'].rootAssembly.referencePoints.items():
            self.rp1 = i         #将第一个参考点的调用序号存储进变量rp1

        # 建立第二个参考点
        mdb.models['Model-1'].rootAssembly.ReferencePoint(
            point=(0.0, 0.0, self.lengthOFpipe * 0.5))
        for i,j in mdb.models['Model-1'].rootAssembly.referencePoints.items():
            if i != self.rp1:
                self.rp2 = i     #将第二个参考点的调用序号存储进变量rp2

        #将参考点和端面耦合
        mdb.models['Model-1'].rootAssembly.Set(name='m_Set-RP1', referencePoints=(
            mdb.models['Model-1'].rootAssembly.referencePoints[self.rp1], ))

        mdb.models['Model-1'].rootAssembly.Surface(name='s_Surf-1', side1Faces=
            mdb.models['Model-1'].rootAssembly.instances['Test-1'].faces.findAt(
            (((self.diaOFpipe - self.thiOFpipe)*0.5,0.0, -self.lengthOFpipe * 0.5), )))

        mdb.models['Model-1'].Coupling(
            controlPoint=mdb.models['Model-1'].rootAssembly.sets['m_Set-RP1'],
            couplingType=KINEMATIC, influenceRadius=WHOLE_SURFACE,localCsys=None,
            name='coupling-rp1',
            surface=mdb.models['Model-1'].rootAssembly.surfaces['s_Surf-1'],
            u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON)

        mdb.models['Model-1'].rootAssembly.Set(name='m_Set-rp2', referencePoints=(
            mdb.models['Model-1'].rootAssembly.referencePoints[self.rp2], ))

        mdb.models['Model-1'].rootAssembly.Surface(name='s_Surf-2', side1Faces=
            mdb.models['Model-1'].rootAssembly.instances['Test-1'].faces.findAt(
            (((self.diaOFpipe - self.thiOFpipe)*0.5,0.0, self.lengthOFpipe * 0.5), )))

        mdb.models['Model-1'].Coupling(
            controlPoint=mdb.models['Model-1'].rootAssembly.sets['m_Set-rp2'],
            couplingType=KINEMATIC, influenceRadius=WHOLE_SURFACE, localCsys=None,
            name='coupling-rp2',
            surface=mdb.models['Model-1'].rootAssembly.surfaces['s_Surf-2'],
            u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON)

        mdb.models['Model-1'].rootAssembly.Surface(
            name='m_Surf-3',
            side1Faces=mdb.models['Model-1'].rootAssembly.instances['Test-1'].faces.findAt(
            ((0, 0, self.diaOFball/2 - self.thiOFball),),))

        mdb.models['Model-1'].rootAssembly.Surface(
            name='s_Surf-3',
            side1Faces=mdb.models['Model-1'].rootAssembly.instances['LeiBan-1'].faces.findAt(
            ((0, 0, self.diaOFball/2 - self.thiOFball),)))

        mdb.models['Model-1'].Tie(
            adjust=ON,
            master=mdb.models['Model-1'].rootAssembly.surfaces['m_Surf-3'],
            name='Tie-1',positionToleranceMethod=COMPUTED,
            slave=mdb.models['Model-1'].rootAssembly.surfaces['s_Surf-3'],
            thickness=ON,tieRotations=ON)

    def BC(self):

        mdb.models['Model-1'].HistoryOutputRequest(
            createStepName='load', name='H-Output-load', rebar=EXCLUDE,
            region= mdb.models['Model-1'].rootAssembly.sets['m_Set-rp2'],
            sectionPoints=DEFAULT, variables=('U3', 'RF3', 'RM2'))
        mdb.models['Model-1'].historyOutputRequests['H-Output-1'].suppress()

        mdb.models['Model-1'].rootAssembly.Set(name='Set-3', referencePoints=(
            mdb.models['Model-1'].rootAssembly.referencePoints[self.rp1], ))

        if self.rigid == False:
            mdb.models['Model-1'].DisplacementBC(
                amplitude=UNSET, createStepName='Initial',
                distributionType=UNIFORM, fieldName='', localCsys=None,
                name='BC-rp1',
                region=mdb.models['Model-1'].rootAssembly.sets['Set-3'],
                u1=SET, u2=SET,u3=SET, ur1=SET, ur2=UNSET, ur3=SET)
        if self.rigid == True:
            mdb.models['Model-1'].DisplacementBC(
                amplitude=UNSET, createStepName='Initial',
                distributionType=UNIFORM, fieldName='', localCsys=None,
                name='BC-rp1',
                region=mdb.models['Model-1'].rootAssembly.sets['Set-3'],
                u1=SET, u2=SET, u3=SET, ur1=SET, ur2=SET, ur3=SET)

        mdb.models['Model-1'].rootAssembly.Set(name='Set-4', referencePoints=(
            mdb.models['Model-1'].rootAssembly.referencePoints[self.rp2], ))

        if self.rigid == False:
            mdb.models['Model-1'].DisplacementBC(
                amplitude=UNSET, createStepName='Initial',
                distributionType=UNIFORM, fieldName='', localCsys=None,
                name='BC-rp2',
                region=mdb.models['Model-1'].rootAssembly.sets['Set-4'],
                u1=SET, u2=SET,u3=UNSET, ur1=SET, ur2=UNSET, ur3=SET)

        if self.rigid == True:
            mdb.models['Model-1'].DisplacementBC(
                amplitude=UNSET, createStepName='Initial',
                distributionType=UNIFORM, fieldName='', localCsys=None,
                name='BC-rp2',
                region=mdb.models['Model-1'].rootAssembly.sets['Set-4'],
                u1=SET, u2=SET, u3=UNSET, ur1=SET, ur2=SET, ur3=SET)

        mdb.models['Model-1'].TabularAmplitude(
            data=((0.0, 0.0), (1.0, self.load)),
            name='Amp-0-' + str(self.load),
            smooth=SOLVER_DEFAULT, timeSpan=STEP)
        mdb.models['Model-1'].rootAssembly.Set(name='Set-5', referencePoints=(
            mdb.models['Model-1'].rootAssembly.referencePoints[self.rp2], ))
        mdb.models['Model-1'].DisplacementBC(amplitude='Amp-0-' + str(self.load), createStepName=
            'load', distributionType=UNIFORM, fieldName='', fixed=OFF, localCsys=None,
            name='Load-rp2', region=mdb.models['Model-1'].rootAssembly.sets['Set-5'],
            u1=UNSET, u2=UNSET, u3=1.0, ur1=UNSET, ur2=UNSET, ur3=UNSET)


    def mesh(self):
        r1 = self.diaOFball*0.5
        r2 = self.diaOFpipe*0.5
        th1 = self.thiOFball
        th2 = self.thiOFpipe
        ll = self.lengthOFpipe
        mdb.models['Model-1'].parts['Test'].PartitionCellByExtendFace(
            cells=mdb.models['Model-1'].parts['Test'].cells,
            extendFace=mdb.models['Model-1'].parts['Test'].faces.findAt(
            (0.0,0.0,r1), ))

        mdb.models['Model-1'].parts['Test'].PartitionCellByExtendFace(
            cells=mdb.models['Model-1'].parts['Test'].cells.findAt(
            ((0.0,0.0,r1), )),
            extendFace=mdb.models['Model-1'].parts['Test'].faces.findAt(
            (0.0,r2,r1), ))

        mdb.models['Model-1'].parts['Test'].PartitionCellByExtendFace(
            cells=mdb.models['Model-1'].parts['Test'].cells.findAt(
            ((0.0,0.0,r1), ),
            ((0.0,0.0,-r1), ), ),
            extendFace=mdb.models['Model-1'].parts['Test'].faces.findAt(
            (0.0,r2-th2,r1), ))

        mdb.models['Model-1'].parts['Test'].PartitionCellByPlaneThreePoints(
            cells=mdb.models['Model-1'].parts['Test'].cells.findAt(
            ((0.0,r1,0), )),
            point1=(100,0,0),
            point2=(0,100,0),
            point3=(100,100,0))



        mdb.models['Model-1'].parts['Test'].PartitionCellByPlaneThreePoints(
            cells = mdb.models['Model-1'].parts['Test'].cells,
            point1 = (100, 0, 100),
            point2 = (-100, 0, 100),
            point3 = (100, 0, -100))

        mdb.models['Model-1'].parts['Test'].PartitionCellByPlaneThreePoints(
            cells=mdb.models['Model-1'].parts['Test'].cells,
            point1=(0, 100, 100),
            point2=(0, -100, 100),
            point3=(0, 100, -100))

        hh1= r2 - th2 * 0.5
        zz1= (r1**2 - hh1**2) ** 0.5
        zz2= ((r1-th1)**2 - hh1**2) ** 0.5
        mdb.models['Model-1'].parts['Test'].Set(edges=
            mdb.models['Model-1'].parts['Test'].edges.findAt(
            ((0,r2 - th2*0.5,ll*0.5), ),            # 1
            ((r2 - th2*0.5,0,ll*0.5), ),            # 2
            ((0, - r2 + th2*0.5,ll*0.5), ),         # 3
            (( - r2 + th2*0.5,0,ll*0.5), ),         # 4
            ((0,r2 - th2*0.5, - ll*0.5), ),         # 5
            ((r2 - th2*0.5,0, - ll*0.5), ),         # 6
            ((0, - r2 + th2*0.5, - ll*0.5), ),      # 7
            (( - r2 + th2*0.5,0, - ll*0.5), ),      # 8

            ((0, hh1, zz1), ),                      # 9
            ((0,-hh1, zz1), ),                      # 10
            ((hh1, 0, zz1), ),                      # 11
            ((-hh1,0, zz1), ),                      # 12
            ((0, hh1,-zz1), ),                      # 13
            ((0,-hh1,-zz1), ),                      # 14
            ((hh1, 0,-zz1), ),                      # 15
            ((-hh1,0,-zz1), ),                      # 16

            ((0, hh1, zz2), ),                      # 17
            ((0,-hh1, zz2), ),                      # 18
            ((hh1, 0, zz2), ),                      # 19
            ((-hh1,0, zz2), ),                      # 20
            ((0, hh1,-zz2), ),                      # 21
            ((0,-hh1,-zz2), ),                      # 22
            ((hh1, 0,-zz2), ),                      # 23
            ((-hh1,0,-zz2), ),                      # 24
            ), name='Set-PipeEdge')

        zz3 = ((r1-th1)**2 - r2**2) ** 0.5
        zz4 = (r1**2 - r2**2) ** 0.5
        zz5 = (zz3 + zz4) * 0.5
        zz6 = ((r1-th1)**2 - (r2-th2)**2) ** 0.5
        zz7 = (r1**2 - (r2-th2)**2) ** 0.5
        zz8 = (zz6 + zz7) * 0.5
        mdb.models['Model-1'].parts['Test'].Set(edges=
            mdb.models['Model-1'].parts['Test'].edges.findAt(
            (( r1 - th1 * 0.5, 0 , 0), ),           # 1
            ((-r1 + th1 * 0.5, 0 , 0), ),           # 2
            (( 0, r1 - th1 * 0.5 , 0), ),           # 3
            (( 0,-r1 + th1 * 0.5 , 0), ),           # 4

            (( r2,    0, zz5), ),                   # 5
            ((-r2,    0, zz5), ),                   # 6
            (( r2-th2,0, zz8),),                    # 7
            ((-r2+th2,0, zz8), ),                   # 8
            (( r2,    0,-zz5), ),                   # 9
            ((-r2,    0,-zz5), ),                   # 10
            (( r2-th2,0,-zz8),),                    # 11
            ((-r2+th2,0,-zz8), ),                   # 12

            ((0, r2,     zz5), ),                   # 13
            ((0,-r2,     zz5), ),                   # 14
            ((0, r2-th2, zz8),),                    # 15
            ((0,-r2+th2, zz8), ),                   # 16
            ((0, r2,    -zz5), ),                   # 17
            ((0,-r2,    -zz5), ),                   # 18
            ((0, r2-th2,-zz8),),                    # 19
            ((0,-r2+th2,-zz8), ),                   # 20

            ((0,0, r1-th1*0.5), ),                  #21
            ((0,0,-r1+th1*0.5), ), ),               #22
            name='Set-BallEdge')

        mdb.models['Model-1'].parts['Test'].seedEdgeByNumber(
            constraint=FINER,
            edges=mdb.models['Model-1'].parts['Test'].sets['Set-PipeEdge'].edges,
            number=self.numOFpipe)

        mdb.models['Model-1'].parts['Test'].seedEdgeByNumber(
            constraint=FINER,
            edges=mdb.models['Model-1'].parts['Test'].sets['Set-BallEdge'].edges,
            number=self.numOFball)

        mdb.models['Model-1'].parts['Test'].seedPart(
            deviationFactor=0.1,
            minSizeFactor=0.1,
            size=self.numOFglobal)

        mdb.models['Model-1'].parts['Test'].generateMesh()
        mdb.models['Model-1'].rootAssembly.regenerate()

        r3 = r1 - th1
        mdb.models['Model-1'].parts['LeiBan'].PartitionCellByPlaneThreePoints(
            cells=mdb.models['Model-1'].parts['LeiBan'].cells.findAt(
            ((0,0,r3),)),
            point1=(100,0,0),
            point2=(0,100,0),
            point3=(100,100,0),)
        mdb.models['Model-1'].parts['LeiBan'].PartitionCellByPlaneThreePoints(
            cells=mdb.models['Model-1'].parts['LeiBan'].cells.findAt(
                ((0,0,r3),),
                ((0,0,-r3),), ),
            point1=(0,100,0),
            point2=(0,0,100),
            point3=(0,100,100),)
        mdb.models['Model-1'].parts['LeiBan'].seedEdgeByNumber(
            constraint=FINER,
            edges=mdb.models['Model-1'].parts['LeiBan'].edges.findAt(
                ((0, 0,  r3),),
                ((0, 0, -r3),),
                (( r3, 0, 0),),
                ((-r3, 0, 0),), ),
            number=self.numOFball)
        mdb.models['Model-1'].parts['LeiBan'].seedPart(
            deviationFactor=0.1,minSizeFactor=0.1,
            size=self.numOFglobal)
        mdb.models['Model-1'].parts['LeiBan'].generateMesh()
        mdb.models['Model-1'].rootAssembly.regenerate()

    def job(self):
        if self.jobname == 'WHSJ_WSR_job01':
            self.jobname = ('Job-WHSJ-WSR' + str(int(self.diaOFball)) + '-' + str(int(self.thiOFball))
            + '-' + str(int(self.diaOFpipe)) + '-' + str(int(self.thiOFpipe)))
        mdb.Job(atTime=None, contactPrint=OFF, description='', echoPrint=OFF,
            explicitPrecision=SINGLE, getMemoryFromAnalysis=True, historyPrint=OFF,
            memory=90, memoryUnits=PERCENTAGE, model='Model-1', modelPrint=OFF,
            multiprocessingMode=DEFAULT, name=self.jobname, nodalOutputPrecision=SINGLE,
            numCpus=8, numDomains=8, numGPUs=0, queue=None, resultsFormat=ODB, scratch=
            '', type=ANALYSIS, userSubroutine='', waitHours=0, waitMinutes=0)
        #生成inp文件
        mdb.Job(name=self.jobname, model='Model-1').writeInput(consistencyChecking=ON)
        # mdb.jobs['Job-1'].submit(consistencyChecking=OFF) # 提交job

def main(d1,d2,d3,t1,t2,l,n1,n2,n3,dis,steel,
         E,v,fy,fu,ey,eu,ininc,maxinc,mininc,numinc,rigid,jobname):
    # 接下来用一个函数来涵盖整个 class ，方便后期制作插件 RSG
    # ABAQUS内置的简单界面RSG制作，仅能对接一个界面，一个函数


    print('Python')
    cs = CS(d1,d2,d3,t1,t2,l,n1,n2,n3,dis,steel,
            E,v,fy,fu,ey,eu,ininc,maxinc,mininc,numinc,rigid,jobname)
    cs.part()
    cs.material()
    cs.assembly()
    cs.step()
    cs.interaction()
    cs.BC()
    cs.mesh()
    cs.job()
    print('completed')

# 为 main() 函数制作 RSG 界面
if __name__ == "__main__":      # 该 if 的含义是：仅在直接执行本脚本时执行下列语句，
                                # 使得本程序可以直接在abaqus中作为脚本运行
                                # 在使用RSG时，对main函数的调用是跨文件的，故下述不会被调用
    main(d1=900,d2=351,d3=200,t1=25,t2=50,l=1950,n1=4,n2=5,n3=15,
         dis=30,steel='Q355',E = 206000,v=0.3,fy=355,fu=650,ey=0.02,eu=0.25,
         ininc=0.01,maxinc=0.1,mininc=1e-7,numinc=1e7,rigid=False,jobname='WHSJ_WS_job01')
