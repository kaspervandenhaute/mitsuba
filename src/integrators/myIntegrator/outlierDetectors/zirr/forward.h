/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2014 by Wenzel Jakob and others.

    Mitsuba is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Mitsuba is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#if !defined(__MITSUBA_FORWARD_PATHS_H_)
#define __MITSUBA_FORWARD_PATHS_H_

#include <mitsuba/bidir/path.h>
#include <mitsuba/bidir/vertex.h>
#include <mitsuba/bidir/edge.h>
#include <mitsuba/bidir/mempool.h>

MTS_NAMESPACE_BEGIN

inline Spectrum evalCached(PathVertex const& v, ETransportMode m) {
	return v.weight[m] * v.pdf[m];
}
inline Spectrum evalCached(PathEdge const& e, ETransportMode m) {
	return e.weight[m] * e.pdf[m];
}
inline Float toShadingG(PathVertex const& v, PathEdge const& e) {
	if (v.isOnSurface())
		return absDot(v.getShadingNormal(), e.d) / absDot(v.getGeometricNormal(), e.d);
	return 1;
}
inline Float remGeometricG(PathEdge const& e, PathVertex const& v) {
	if (v.isOnSurface())
		return 1 / absDot(v.getGeometricNormal(), e.d);
	return 1;
}
inline Float toAreaMeasure(Float pdf, EMeasure measure, PathEdge const& e, PathVertex const& vn) {
	if (measure != EArea && e.length != 0.0f) {
		pdf /= e.length * e.length;
		if (vn.isOnSurface())
			pdf *= absDot(e.d, vn.getGeometricNormal());
	}
	return pdf;
}
inline Float toAngleMeasure(Float pdf, EMeasure measure, PathEdge const& e, PathVertex const& vn) {
	if (measure == EArea && e.length != 0.0f) {
		pdf *= e.length * e.length;
		if (vn.isOnSurface())
			pdf /= absDot(e.d, vn.getGeometricNormal());
	}
	return pdf;
}

inline void extend(Path& path, int hitIndex, PathVertex* v) {
	if (hitIndex < path.vertexCount())
		path.vertex(hitIndex) = v;
	else
		path.append(v);
}
inline void extend(Path& path, int hitIndex, PathEdge* e) {
	if (hitIndex < path.edgeCount())
		path.edge(hitIndex) = e;
	else
		path.append(e);
}
inline void extend(Path& path, int hitIndex, PathVertex* v, PathEdge* e) {
	extend(path, hitIndex, v);
	extend(path, hitIndex, e);
}
inline void extend(Path& path, int hitIndex, PathEdge* e, PathVertex* v) {
	path.extend(hitIndex + 2);
	path.edge(hitIndex) = e;
	path.vertex(hitIndex + 1) = v;
}
inline void extend(Path& path, int hitIndex, PathVertex* v, PathEdge* e, PathVertex* vn) {
	path.extend(hitIndex + 2);
	path.vertex(hitIndex) = v;
	path.edge(hitIndex) = e;
	path.vertex(hitIndex + 1) = vn;
}

inline void extendLazy(Path& path, int hitIndex, const Path& spath) {
	int vc = path.vertexCount();
	int ec = path.edgeCount();
	path.extend(hitIndex + 1);
	for (; vc <= hitIndex; ++vc)
		path.vertex(vc) = spath.vertex(vc);
	for (; ec < hitIndex; ++ec)
		path.edge(ec) = spath.edge(ec);
}

inline bool isEmitting(PathVertex const& v) {
	return v.isSurfaceInteraction() && v.getIntersection().isEmitter(); // fixme: Mitsuba has no emitting volumes so far?
}
inline bool isSensing(PathVertex const& v) {
	return v.isSurfaceInteraction() && v.getIntersection().isSensor();
}

inline bool isRefraction(Vector const& exitant, PathVertex const& v, Vector const& incident) {
	if (v.isSurfaceInteraction()) {
		auto n = v.getGeometricNormal();
		return dot(n, exitant) * dot(n, incident) < Float(0);
	}
	else
		return false;
}
inline Float bsdfEta(PathVertex const& v) {
	Float eta = 1;
	if (v.isSurfaceInteraction()) {
		return v.getIntersection().getBSDF()->getEta();
	}
	return eta;
}
inline Float refractionEta(Vector const& exitant, PathVertex const& v) {
	Float eta = 1;
	if (v.isSurfaceInteraction()) {
		eta = v.getIntersection().getBSDF()->getEta();
		auto n = v.getGeometricNormal();
		if (dot(n, exitant) < Float(0))
			eta = 1 / eta;
	}
	return eta;
}
inline bool interactionEta(Vector const& exitant, PathVertex const& v, Vector const& incident) {
	Float eta = 1;
	if (v.isSurfaceInteraction()) {
		auto n = v.getGeometricNormal();
		if (dot(n, exitant) * dot(n, incident) < Float(0)) {
			eta = v.getIntersection().getBSDF()->getEta();
			if (dot(n, exitant) < Float(0))
				eta = 1 / eta;
		}
	}
	return eta;
}

inline Spectrum emission(PathVertex const& v, Vector const& d) {
	Spectrum Le = Spectrum(0.0f);
	if (v.isSurfaceInteraction() && v.getIntersection().isEmitter())
		Le = v.getIntersection().Le(d);
	return Le;
}

inline bool isScatteringAndEmissive(PathVertex const& vertex, bool& emitter, bool andEmissive = true) {
	const BSDF* bsdf = nullptr;
	switch (vertex.getType()) {
		case PathVertex::EMediumInteraction:
			emitter = false;
			return !andEmissive;
		case PathVertex::ESurfaceInteraction: {
			const Intersection& isec = vertex.getIntersection();
			bsdf = isec.getBSDF();
			emitter = isec.isEmitter();
			break;
		}
		case PathVertex::EEmitterSample: {
			emitter = true;
			AbstractEmitter const* emitter = vertex.getAbstractEmitter();
			// no interaction on degenerate emitters
			if (emitter->isDegenerate())
				return false;
			Shape const* shape = emitter->getShape();
			if (!shape)
				return false;
			bsdf = shape->getBSDF();
			break;
		}
		default:
			emitter = false;
			return false;
	}
	return bsdf && bsdf->hasComponent(BSDF::EAll) && (!andEmissive || emitter);
}

inline void preEvalInteraction(Vector const& exitant, PathVertex& vertex, Vector const& incident, ETransportMode mode = ERadiance, EMeasure measure = ESolidAngle) {
	BDAssert(measure == ESolidAngle || measure == EDiscrete);
	switch (vertex.getType()) {
		case PathVertex::ESurfaceInteraction: {
				const Intersection &its = vertex.getIntersection();
				const BSDF *bsdf = its.getBSDF();

				BSDFSamplingRecord bRec(its, its.toLocal(exitant), its.toLocal(incident), mode);
				vertex.weight[mode] = bsdf->eval(bRec, measure);
				vertex.pdf[mode] = -1.0f;
				vertex.measure = measure;
				vertex.componentType = (measure == EDiscrete ? BSDF::EDelta : BSDF::ESmooth)
					& (bRec.wi.z * bRec.wo.z >= 0 ? BSDF::EReflection : BSDF::ETransmission);
			}
			break;

		case PathVertex::EMediumInteraction: {
				BDAssert(measure == ESolidAngle);

				const MediumSamplingRecord &mRec = vertex.getMediumSamplingRecord();
				const PhaseFunction *phase = mRec.medium->getPhaseFunction();
				PhaseFunctionSamplingRecord pRec(mRec, exitant, incident, mode);
				vertex.weight[mode] = mRec.sigmaS * phase->eval(pRec);
				vertex.pdf[mode] = -1.0f;
				vertex.measure = measure;
			}
			break;

	default:
		BDAssert(false);
	}
}

inline void updateInteraction(Vector const& exitant, PathVertex& vertex, Vector const& incident, ETransportMode mode = ERadiance, EMeasure measure = ESolidAngle) {
	BDAssert(measure == ESolidAngle || measure == EDiscrete);
	switch (vertex.getType()) {
		case PathVertex::ESurfaceInteraction: {
				const Intersection &its = vertex.getIntersection();
				const BSDF *bsdf = its.getBSDF();

				BSDFSamplingRecord bRec(its, its.toLocal(exitant), its.toLocal(incident), mode);
				Float pdf = bsdf->pdf(bRec, measure);
				
#ifdef BIDIR_DISCRETE_PMF_TRACKING
				vertex.choicePMF[mode] = bRec.choicePMF;
#endif
				if (pdf > 0.0f) {
					if (vertex.pdf[mode] != -1.0f)
						vertex.weight[mode] = bsdf->eval(bRec, measure);
					vertex.weight[mode] /= pdf;
				}
				else
					vertex.weight[mode] = Spectrum(0.0f);
				vertex.pdf[mode] = pdf;
				vertex.measure = measure;
				vertex.componentType = (measure == EDiscrete ? BSDF::EDelta : BSDF::ESmooth)
					& (bRec.wi.z * bRec.wo.z >= 0 ? BSDF::EReflection : BSDF::ETransmission);
			}
			break;

		case PathVertex::EMediumInteraction: {
				BDAssert(measure == ESolidAngle);

				const MediumSamplingRecord &mRec = vertex.getMediumSamplingRecord();
				const PhaseFunction *phase = mRec.medium->getPhaseFunction();
				PhaseFunctionSamplingRecord pRec(mRec, exitant, incident, mode);
				Float pdf = phase->pdf(pRec);

				if (pdf > 0.0f) {
					if (vertex.pdf[mode] != -1.0f)
						vertex.weight[mode] = mRec.sigmaS * phase->eval(pRec);
					vertex.weight[mode] /= pdf;
				}
				else
					vertex.weight[mode] = Spectrum(0.0f);
				vertex.pdf[mode] = pdf;
				vertex.measure = measure;
#ifdef BIDIR_DISCRETE_PMF_TRACKING
				vertex.choicePMF[mode] = 1.0f;
#endif
			}
			break;

	default:
		BDAssert(false);
	}
}

inline void cacheNEE(Scene const* scene, PathVertex const& connectVertex, PathVertex& emitterSample, PathVertex& emitter, bool lazy = false) {
	if (lazy && (emitterSample.componentType & BSDF::EUsesSampler))
		return;
	EMeasure neeMeasure = (emitter.measure == EDiscrete) ? EDiscrete : emitterSample.measure;
	if (neeMeasure != EDiscrete) neeMeasure = ESolidAngle; // always cache in solid angle
	emitter.pdf[ERadiance] = connectVertex.evalPdfDirect(scene, &emitterSample, EImportance, neeMeasure);
	emitterSample.componentType |= BSDF::EUsesSampler | (neeMeasure == EDiscrete ? BSDF::EDelta : BSDF::ESmooth);
}

inline Spectrum collectEmitter(Scene const* scene, const Vector3 &exitant, PathVertex const* emitterSource, PathVertex* emitterSample, PathEdge* emitterEdge, PathVertex* emitter) {
	Spectrum Le = emission(*emitterSource, exitant);
	if (Le.isZero())
		return Le;

	if (emitterSource != emitterSample)
		*emitterSample = *emitterSource;
	if (!emitterSample->cast(scene, PathVertex::EEmitterSample)) {
		Le = Spectrum(0.0f);
		return Le;
	}

	emitterSample->weight[ERadiance] = Spectrum(1.0f);
	emitterSample->weight[EImportance] = Spectrum(0.0f);
	emitterSample->pdf[ERadiance] = 1.0f;
	emitterSample->pdf[EImportance] = 0.0f;
	emitterSample->rrWeight = 1.0f;
	emitterSample->componentType = 0;
	emitterEdge->weight[ERadiance] = Spectrum(0.0f);
	emitterEdge->weight[EImportance] = Spectrum(1.0f);
	emitterEdge->pdf[ERadiance] = 0.0f;
	emitterEdge->pdf[EImportance] = 1.0f;
	emitterEdge->medium = NULL;
	emitterEdge->d = Vector3(0.0f);
	emitterEdge->length = 0.0f;
	emitter->makeEndpoint(scene, emitterSample->getTime(), EImportance);
	emitter->weight[ERadiance] = Spectrum(0.0f);
	emitter->weight[EImportance] = Le;
	emitter->pdf[ERadiance] = 0.0f;
	emitter->pdf[EImportance] = 1.f;
	emitter->rrWeight = 1.0f;

	emitter->measure = emitterSample->measure; // note: should always be EArea
	emitterSample->measure = (emitterSample->degenerate) ? EDiscrete : ESolidAngle;

	return Le;
}
inline Spectrum extendEmitter(Scene const* scene, Path& path, int hitIndex, PathVertex* emitterSample, PathEdge* emitterEdge, PathVertex* emitter
	, bool updateCached = true, bool neePdf = false) {	
	if (emitterSample != path.vertex(hitIndex))
		*emitterSample = *path.vertex(hitIndex);

	Spectrum Le;
	if (!emitterSample->isEmitterSample()) {
		PathEdge const& connectEdge = *path.edge(hitIndex - 1);
		Le = collectEmitter(scene, connectEdge.d, emitterSample, emitterSample, emitterEdge, emitter);
		if (Le.isZero())
			return Le;
	} else {
		BDAssert(emitter->isEmitterSupernode());
		Le = evalCached(*emitter, EImportance) * emitterSample->weight[ERadiance];
	}

	extend(path, hitIndex, emitterSample, emitterEdge, emitter);

	PathVertex const& connectVertex = *path.vertex(hitIndex - 1);
	// note: vertices [hitIndex, ...] outdated!
	if (updateCached) {
		// backup cached info
		uint16_t cT = emitterSample->componentType;
		Float cPdf = emitter->pdf[ERadiance];
		// bidi info
		bool su = emitter->update(scene, nullptr, emitterSample, EImportance, emitter->measure);
		su &= emitterSample->update(scene, emitter, &connectVertex, EImportance, emitterSample->measure == EDiscrete ? EDiscrete : EArea);
		if (!su) {
			Le = Spectrum(0.0f);
			return Le;
		}
		// restore
		emitterSample->componentType = cT;
		emitter->pdf[ERadiance] = cPdf;
	}

	// NEE pdf for MIS
	if (neePdf)
		cacheNEE(scene, connectVertex, *emitterSample, *emitter, /*lazy*/ true);

	return Le;
}

inline Spectrum collectNEE(Scene const* scene, Sampler& sampler, const PathEdge &exitant, PathVertex const* connectSource, PathVertex* connectVertex
	, PathEdge* connectEdge, int& connectInteractions
	, PathVertex* emitterSample, PathEdge* emitterEdge, PathVertex* emitter
	, bool updateLight = true, bool allowDegenerate = false, bool omitConnectionWeights = false) {
	// sample emitter
	Spectrum Le;
	{
		PathVertex* connectS = const_cast<PathVertex*>(connectSource);
		bool degenerate = connectS->degenerate;
		if (allowDegenerate)
			connectS->degenerate = false;
		Le = connectS->sampleDirect(scene, &sampler, emitter, emitterEdge, emitterSample, EImportance, updateLight);
		if (allowDegenerate)
			connectS->degenerate = degenerate;
	}
	if (Le.isZero())
		return Le;

	// determine sampled event type
	bool isTransmission = false;
	if (connectSource->isSurfaceInteraction()) {
		Intersection const& connectIts = connectSource->getIntersection();
		Vector wi = normalize(emitterSample->getPosition() - connectIts.p);
		Vector wo = exitant.d;

		Float wiDotShnN = dot(connectIts.shFrame.n, wi),
			woDotShnN = dot(connectIts.shFrame.n, wo);
		Float wiDotGeoN = dot(connectIts.geoFrame.n, wi),
			woDotGeoN = dot(connectIts.geoFrame.n, wo);
		if (wiDotShnN * wiDotGeoN > 0 && woDotShnN * woDotGeoN > 0)
			isTransmission = (wiDotShnN * woDotShnN < 0);
		else {
			Le = Spectrum(0.0f);
			return Le;
		}
	}

	// connect
	bool isVisible = connectEdge->pathConnectAndCollapse(scene, emitterEdge, emitterSample, connectSource, &exitant, connectInteractions);
	if (!isVisible) {
		Le = Spectrum(0.0f);
		return Le;
	}

	emitterSample->weight[ERadiance] = Spectrum(1.0f);
	emitterSample->pdf[ERadiance] = 1.0f;
	emitterSample->rrWeight = 1.0f;
	emitterSample->measure = (emitterSample->measure == EDiscrete) ? EDiscrete : ESolidAngle;
	emitter->weight[ERadiance] = Le;
	emitter->rrWeight = 1.0f;

	if (connectSource != connectVertex)
		*connectVertex = *connectSource;
	memset(connectVertex->weight, 0, sizeof(connectVertex->weight));
	memset(connectVertex->pdf, 0, sizeof(connectVertex->pdf));
	connectVertex->componentType = BSDF::ESmooth & (isTransmission ? BSDF::ETransmission : BSDF::EReflection);
	connectVertex->rrWeight = 1.f;

	// Classic NEE
	if (!omitConnectionWeights) {
		preEvalInteraction(exitant.d, *connectVertex, -connectEdge->d);
		Spectrum connectWgt = connectVertex->weight[ERadiance];
		connectWgt *= connectEdge->evalCached(emitterSample, connectVertex, PathEdge::ETransmittance);
		Le *= connectWgt;
		if (allowDegenerate && Le.isZero())
			Le /= Spectrum(0.0f); // NaN to indicate failure due to weights
	}

	return Le;
}

inline Spectrum extendNEE(Scene const* scene, Sampler* sampler, Path& path
	, int connectIdx, PathVertex* connectVertex, PathEdge* connectEdge, int* connectInteractions
	, PathVertex* emitterSample, PathEdge* emitterEdge, PathVertex* emitter
	, bool updateCached = true, bool allowDegenerate = false
	, bool omitConnectionWeights = false, bool neePdf = false) {
	if (connectVertex != path.vertex(connectIdx))
		*connectVertex = *path.vertex(connectIdx);
	Spectrum Le;
	if (sampler) {
		int tmp = 0;
		if (!connectInteractions)
			connectInteractions = &tmp;
		Le = collectNEE(scene, *sampler, *path.edge(connectIdx - 1)
			, connectVertex, connectVertex, connectEdge, *connectInteractions
			, emitterSample, emitterEdge, emitter
			, updateCached, allowDegenerate, true);
		if (!Le.isPositive())
			return Le;
	}
	else {
		Le = emitter->weight[ERadiance];
	}
	
	path.extend(connectIdx + 3);
	path.vertex(connectIdx) = connectVertex;
	path.edge(connectIdx) = connectEdge;
	path.vertex(connectIdx+1) = emitterSample;
	path.edge(connectIdx+1) = emitterEdge;
	path.vertex(connectIdx+2) = emitter;

	// note: vertices [connectIdx, ...] outdated!
	if (updateCached) {
		// backup cached info
		uint16_t cT = emitterSample->componentType;
		// bidir info
		bool su = emitterSample->update(scene, emitter, connectVertex, EImportance, emitterSample->measure == EDiscrete ? EDiscrete : EArea);
		su &= connectVertex->update(scene, emitterSample, path.vertex(connectIdx - 1), EImportance, EArea) | allowDegenerate;
		if (!su) {
			Le = Spectrum(0.0f);
			return Le;
		}
		// restore
		emitterSample->componentType = cT;
	}
	else if (!omitConnectionWeights)
		updateInteraction(path.edge(connectIdx-1)->d, *connectVertex, -connectEdge->d);

	// NEE pdf for MIS
	if (neePdf)
		cacheNEE(scene, *connectVertex, *emitterSample, *emitter, /*lazy*/ true);

	// Classic NEE
	if (!omitConnectionWeights) {
		Spectrum connectWgt = connectVertex->weight[ERadiance] * connectVertex->pdf[ERadiance];
		if (updateCached)
			connectWgt *= toAngleMeasure(1.0f, connectVertex->measure, *connectEdge, *emitterSample);
		connectWgt *= connectEdge->evalCached(emitterSample, connectVertex, PathEdge::ETransmittance);
		Le *= connectWgt;
		if (allowDegenerate && Le.isZero())
			Le /= Spectrum(0.0f); // NaN to indicate failure due to weights
	}
	
	return Le;
}

inline Spectrum throughputForward(Path const& path, int connectionIdx, ETransportMode mode = ERadiance, bool RR = false) {
	Spectrum t = Spectrum(1.f);
	for (int i = 0; i < connectionIdx; ++i) {
		auto v = path.vertex(i);
		t *= v->weight[mode] * path.edge(i)->weight[mode];
		if (RR)
			t *= std::max(v->rrWeight, 1.f);
	}
	return t;
}

inline Spectrum throughputBackward(Path const& path, int connectionIdx, ETransportMode mode = ERadiance, bool RR = false) {
	int vc = (int)path.vertexCount();
	Spectrum t = Spectrum(1.f);
	for (int i = vc - 1; i > connectionIdx; --i) {
		auto v = path.vertex(i);
		t *= v->weight[mode] * path.edge(i - 1)->weight[mode];
		if (RR)
			t *= std::max(v->rrWeight, 1.f);
	}
	return t;
}

inline Float recomputeRRBackwards(Path const& path, int connectionIdx, int rrDepth = 1, ETransportMode mode = ERadiance) {
	int vc = (int)path.vertexCount();
	Spectrum t = Spectrum(1.f);
	Float rr = 1.0f;
	for (int i = vc - 1, depth = -1; i > connectionIdx; --i, ++depth) {
		auto v = path.vertex(i);
		t *= v->weight[mode];
		if (depth >= rrDepth) {
			rr *= PathVertex::getRRProbability(t);
			t /= rr;
		}
		t *= path.edge(i - 1)->weight[mode];
	}
	return rr;
}

inline PathVertex const* connectVertexForward(Path const& path, PathEdge& edge, PathVertex const*& succOrSample
	, int connectionIdx, ETransportMode mode = ERadiance) {
	for ( ; ; --connectionIdx) {
		PathVertex const* result = path.vertex(connectionIdx);
		if (!(result->componentType & BSDF::ENull))
			return result;
		PathEdge* e = path.edge(connectionIdx - 1);
		edge.pdf[mode] *= result->pdf[mode] * e->pdf[mode];
		edge.length = e->length; // for measure change to succ vertex
		succOrSample = result;
	}
}

inline PathVertex const* connectVertexBackward(Path const& path, PathEdge& edge, PathVertex const*& succOrSample
	, int connectionIdx = 2, ETransportMode mode = ERadiance) {
	for ( ; ; ++connectionIdx) {
		PathVertex const* result = path.vertex(connectionIdx);
		if (!(result->componentType & BSDF::ENull)) {
			succOrSample = path.vertex(connectionIdx - 1);
			return result;
		}
		PathEdge* e = path.edge(connectionIdx);
		edge.pdf[mode] *= result->pdf[mode] * e->pdf[mode];
		edge.length = e->length; // for measure change to succ vertex
	}
}

inline int connectIdxBackward(Path const& path, Float& edgePdf, int connectionIdx = 2, ETransportMode mode = ERadiance) {
	edgePdf = 1.0f;
	for ( ; ; ++connectionIdx) {
		PathVertex const* result = path.vertex(connectionIdx);
		if (!(result->componentType & BSDF::ENull))
			return connectionIdx;
		PathEdge* e = path.edge(connectionIdx);
		edgePdf *= result->pdf[mode] * e->pdf[mode];
	}
}

inline Spectrum radianceEmitter(PathEdge const& connectEdge, PathVertex const& emitterSample, PathEdge const& emitterEdge, PathVertex const& emitter) {
	Spectrum rad = emitterSample.weight[ERadiance] * evalCached(emitterEdge, EImportance) * evalCached(emitter, EImportance);
	return rad;
}

inline Spectrum radianceHit(PathVertex const& connectVertex, PathEdge const& connectEdge, PathVertex const& emitterSample, PathEdge const& emitterEdge, PathVertex const& emitter, bool RR = false) {
	Spectrum rad = radianceEmitter(connectEdge, emitterSample, emitterEdge, emitter);
	rad *= connectVertex.weight[ERadiance] * connectEdge.weight[ERadiance];
	if (RR)
		rad *= std::max(connectVertex.rrWeight, 1.f);
	return rad;
}

inline Spectrum radianceGeneric(PathVertex const& connectVertex, PathVertex const& lastVertex, PathEdge const& lastEdge
	, PathVertex const& emitterSample, PathEdge const& emitterEdge, PathVertex const& emitter) {
	Spectrum rad = radianceEmitter(lastEdge, emitterSample, emitterEdge, emitter);
	rad *= connectVertex.pdf[ERadiance] * lastVertex.weight[ERadiance] * evalCached(lastEdge, ERadiance);
	return rad;
}

inline Spectrum radianceNEE(PathVertex const& connectVertex, PathVertex const& lastVertex, PathEdge const& lastEdge
	, PathVertex const& emitterSample, PathEdge const& emitterEdge, PathVertex const& emitter, float neePdf) {
	return radianceGeneric(connectVertex, lastVertex, lastEdge, emitterSample, emitterEdge, emitter) / neePdf;
}
inline Spectrum radianceNEECached(PathVertex const& connectVertex, PathVertex const& lastVertex, PathEdge const& lastEdge
	, PathVertex const& emitterSample, PathEdge const& emitterEdge, PathVertex const& emitter) {
	return radianceNEE(connectVertex, lastVertex, lastEdge, emitterSample, emitterEdge, emitter, emitter.pdf[ERadiance]);
}

struct ForwardPDF {
	float Hit;
	float NEE;
	bool Specular;
	bool DirLight;
	bool PointLight;
	bool SolidAngle;
};
inline ForwardPDF forwardPdfHit(PathVertex const& connectVertex, PathEdge const& connectEdge, PathVertex const& emitterSample, PathVertex const& emitter) {
	ForwardPDF pdf;
	pdf.Specular = (connectVertex.measure == EDiscrete);
	pdf.DirLight = (emitterSample.measure == EDiscrete);
	pdf.PointLight = (emitter.measure == EDiscrete);
	pdf.SolidAngle = (connectVertex.measure != EArea);
	pdf.Hit = (!pdf.DirLight && !pdf.PointLight) ? connectVertex.pdf[ERadiance] * connectEdge.pdf[ERadiance] : 0.0f;
	pdf.NEE = -1.0f;
	return pdf;
}
inline Float forwardPdfNEE(Scene const* scene, ForwardPDF const& pdf
	, PathVertex const& connectVertex, PathEdge const& connectEdge
	, PathVertex const& succOrSample, PathVertex const& emitterSample) {
	if (pdf.Specular || connectVertex.isSensorSample())
		return 0.0f;
	EMeasure neeMeasure = ESolidAngle;
	if (pdf.PointLight || pdf.DirLight)
		neeMeasure = EDiscrete;
	Float neePdf = connectVertex.evalPdfDirect(scene, &emitterSample, EImportance, neeMeasure);
	if (neeMeasure != EDiscrete && !pdf.SolidAngle)
		neePdf = toAreaMeasure(neePdf, ESolidAngle, connectEdge, succOrSample);
	return neePdf;
}

//#define BIDI_DEBUG_CACHES

inline ForwardPDF forwardPdfCached(Scene const* scene, PathVertex const& connectVertex, PathEdge const& connectEdge
	, PathVertex const& succOrSample, PathVertex const& emitterSample, PathVertex const& emitter) {
	ForwardPDF pdf = forwardPdfHit(connectVertex, connectEdge, emitterSample, emitter);
	pdf.NEE = (!pdf.Specular && !connectVertex.isSensorSample()) ? emitter.pdf[ERadiance] : 0.0f;
	if (!pdf.DirLight && !pdf.PointLight && !pdf.SolidAngle)
		pdf.NEE = toAreaMeasure(pdf.NEE, ESolidAngle, connectEdge, succOrSample);
#ifdef BIDI_DEBUG_CACHES
	{
		Float checkPdf = forwardPdfNEE(scene, pdf, connectVertex, connectEdge, succOrSample, emitterSample);
		Float relError = std::abs(checkPdf / pdf.NEE - 1.0f);
		if (std::abs(checkPdf - pdf.NEE) < .001f
			|| relError < .0001f) ;
		else
			SLog(relError < 0.01f ? EWarn : EError, "NEE cache mismatch: %f (cached) vs. %f", pdf.NEE, checkPdf);
	}
#endif
	return pdf;
}
inline ForwardPDF forwardPdf(Scene const* scene, PathVertex const& connectVertex, PathEdge const& connectEdge
	, PathVertex const& succOrSample, PathVertex const& emitterSample, PathVertex const& emitter
	, bool allowCached = true) {
	ForwardPDF pdf = forwardPdfHit(connectVertex, connectEdge, emitterSample, emitter);
	if (allowCached && (emitterSample.componentType & BSDF::EUsesSampler)) {
		pdf.NEE = (!pdf.Specular && !connectVertex.isSensorSample()) ? emitter.pdf[ERadiance] : 0.0f;
		if (!pdf.DirLight && !pdf.PointLight && !pdf.SolidAngle)
			pdf.NEE = toAreaMeasure(pdf.NEE, ESolidAngle, connectEdge, succOrSample);
#ifdef BIDI_DEBUG_CACHES
		{
			Float checkPdf = forwardPdfNEE(scene, pdf, connectVertex, connectEdge, succOrSample, emitterSample);
			BDAssert(checkPdf == pdf.NEE);
		}
#endif
	}
	else
		pdf.NEE = forwardPdfNEE(scene, pdf, connectVertex, connectEdge, succOrSample, emitterSample);
	return pdf;
}

inline ForwardPDF forwardPdfBackward(Scene const* scene, Path const& path, bool allowCached = true, int firstConnectionIdx = 2) {
	PathVertex const* emitterSample = path.vertex(1);
	PathVertex const* succOrSample = emitterSample;
	PathEdge connectEdge = *path.edge(1);
	PathVertex const* connectVertex = connectVertexBackward(path, connectEdge, succOrSample, firstConnectionIdx);
	ForwardPDF pdf = forwardPdf(scene, *connectVertex, connectEdge, *succOrSample, *emitterSample, *path.vertex(0));
	return pdf;
}

MTS_NAMESPACE_END

#endif /* __MITSUBA_FORWARD_PATHS_H_ */
