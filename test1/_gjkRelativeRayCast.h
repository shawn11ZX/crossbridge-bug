

	
	/*
	
		lambda			:	the time of impact(TOI)
		initialLambda	:	the start time of impact value (disable)
		s				:	the sweep ray origin in ConvexB's space
		r				:	the normalized sweep ray direction scaled by the sweep distance. r should be in ConvexB's space
		normal			:	the contact normal in ConvexB's space
		closestA		:	the tounching contact in ConvexB's space
		inflation		:	the amount by which we inflate the swept shape. If the inflated shapes aren't initially-touching, 
							the TOI will return the time at which both shapes are at a distance equal to inflation separated. If inflation is 0
							the TOI will return the time at which both shapes are touching.
	*/
	template<class ConvexA, class ConvexB>
	bool _gjkRelativeRayCast(ConvexA& a, ConvexB& b, const Ps::aos::PsMatTransformV& aToB, const Ps::aos::FloatVArg initialLambda, const Ps::aos::Vec3VArg s, const Ps::aos::Vec3VArg r, Ps::aos::FloatV& lambda, Ps::aos::Vec3V& normal, Ps::aos::Vec3V& closestA, const PxReal _inflation/*, const bool initialOverlap*/)
	{
		PX_UNUSED(initialLambda);

		using namespace Ps::aos;

		const FloatV inflation = FLoad(_inflation);
		const Vec3V zeroV = V3Zero();
		const FloatV zero = FZero();
		const FloatV eps = FEps();
		const FloatV one = FOne();
		const BoolV bTrue = BTTTT();

		const FloatV maxDist = FLoad(PX_MAX_REAL);
	
		FloatV _lambda = zero;//initialLambda;
		Vec3V x = V3ScaleAdd(r, _lambda, s);
		PxU32 size=1;
	
		const Vec3V _initialSearchDir = V3Sel(FIsGrtr(V3Dot(aToB.p, aToB.p), eps), aToB.p, V3UnitX());
		const Vec3V initialSearchDir = V3Normalize(_initialSearchDir);
		

		const Vec3V initialSupportA(a.supportSweepRelative(V3Neg(initialSearchDir), aToB));
		const Vec3V initialSupportB( b.supportSweepLocal(initialSearchDir));
		 
		Vec3V Q[4] = {V3Sub(initialSupportA, initialSupportB), zeroV, zeroV, zeroV}; //simplex set
		Vec3V A[4] = {initialSupportA, zeroV, zeroV, zeroV}; //ConvexHull a simplex set
		Vec3V B[4] = {initialSupportB, zeroV, zeroV, zeroV}; //ConvexHull b simplex set
		 

		Vec3V closest = Q[0];
		Vec3V supportA = initialSupportA;
		Vec3V supportB = initialSupportB;
		Vec3V support = Q[0];

		const FloatV minMargin = FMin(a.getSweepMargin(), b.getSweepMargin());
		const FloatV eps1 = FMul(minMargin, FLoad(0.1f));
		const FloatV inflationPlusEps(FAdd(eps1, inflation));
		const FloatV eps2 = FMul(eps1, eps1);

		const FloatV inflation2 = FMul(inflationPlusEps, inflationPlusEps);

		FloatV sDist = V3Dot(closest, closest);
		FloatV minDist = sDist;
		
		
		BoolV bNotTerminated = FIsGrtr(sDist, eps2);
		BoolV bCon = bTrue;

		Vec3V prevClosest = closest;

		Vec3V nor = closest;
		
		while(BAllEq(bNotTerminated, bTrue))
		{
			printf("Start Loop: sDist.x is %.60f\n", sDist.x); //循环开始 这个时候 sDist 突然变为 0.0506684184074401855468750
			minDist = sDist;
			prevClosest = closest;

			const Vec3V vNorm = V3Neg(V3Normalize(closest));

			supportA=a.supportSweepRelative(vNorm, aToB);
			supportB=V3Add(x, b.supportSweepLocal(V3Neg(vNorm)));
		
			//calculate the support point
			support = V3Sub(supportA, supportB);
			const Vec3V w = V3Neg(support);
			const FloatV vw = FSub(V3Dot(vNorm, w), inflationPlusEps);
			const FloatV vr = V3Dot(vNorm, r);
			if(FAllGrtr(vw, zero))
			{
	
				if(FAllGrtrOrEq(vr, zero))
				{
					return false;
				}
				else
				{
					const FloatV _oldLambda = _lambda;
					_lambda = FSub(_lambda, FDiv(vw, vr));
					if(FAllGrtr(_lambda, _oldLambda))
					{
						if(FAllGrtr(_lambda, one))
						{
							return false;
						}
						const Vec3V bPreCenter = x;
						x = V3ScaleAdd(r, _lambda, s);
			
						const Vec3V offSet =V3Sub(x, bPreCenter);
						const Vec3V b0 = V3Add(B[0], offSet);
						const Vec3V b1 = V3Add(B[1], offSet);
						const Vec3V b2 = V3Add(B[2], offSet);
					
						B[0] = b0;
						B[1] = b1;
						B[2] = b2;

						Q[0]=V3Sub(A[0], b0);
						Q[1]=V3Sub(A[1], b1);
						Q[2]=V3Sub(A[2], b2);

						supportB = V3Add(x, b.supportSweepLocal(V3Neg(vNorm)));
				
						support = V3Sub(supportA, supportB);
						minDist = maxDist;
						nor = closest;
						//size=0;
					}
				}
			}

			PX_ASSERT(size < 4);
			A[size]=supportA;
			B[size]=supportB;
			Q[size++]=support;
	
			//calculate the closest point between two convex hull
			closest = GJKCPairDoSimplex(Q, A, B, support, size);
			sDist = V3Dot(closest, closest);

			bCon = FIsGrtr(minDist, sDist);
			bNotTerminated = BAnd(FIsGrtr(sDist, inflation2), bCon);
			printf("End Loop: sDist.x is %.60f\n", sDist.x); //循环结束， 这个时候 sDist 值为 0.050668418176030627364525571465492248535156250
		}

		const BoolV aQuadratic = a.isMarginEqRadius();
		//ML:if the Minkowski sum of two objects are too close to the original(eps2 > sDist), we can't take v because we will lose lots of precision. Therefore, we will take
		//previous configuration's normal which should give us a reasonable approximation. This effectively means that, when we do a sweep with inflation, we always keep v because
		//the shapes converge separated. If we do a sweep without inflation, we will usually use the previous configuration's normal.
		nor = V3Sel(BAnd(FIsGrtr(sDist, eps2), bCon), closest, nor);
		nor =  V3Normalize(nor);
		normal = nor;
		lambda = _lambda;
		const Vec3V closestP = V3Sel(bCon, closest, prevClosest);
		Vec3V closA = zeroV, closB = zeroV;
		getClosestPoint(Q, A, B, closestP, closA, closB, size);
		closestA = V3Sel(aQuadratic, V3NegScaleSub(nor, a.getMargin(), closA), closA);  
		
		return true;
	}

